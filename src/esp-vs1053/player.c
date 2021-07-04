#include <driver/gpio.h>
#include <errno.h>
#include <esp_log.h>
#include <fcntl.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <string.h>
#include <unistd.h>

#include "private.h"

// The player task is pinned to a core because ISRs must be
// deregistered on the same core as that on which they were registered.
#define RUN_ON_CORE_NUM 1

#define TASK_STACK_SIZE 4096

// How many times we should attempt to send `DATA_CHUNK_SIZE` bytes
// while playing before stopping to check for state updates (e.g.
// new file/cancellation).
#define PLAYER_CHUNK_ITERS 64

// The maximum number of bytes we can safely write if DREQ is high
// (see datasheet).
#define DATA_CHUNK_SIZE 32

// Specified by datasheet, see Subsection 10.5.
#define FINISHING_ENDFILL_BYTES 2052
#define PENDING_CANCEL_GIVE_UP_BYTES 2048

#define ID3v2_HEADER_BYTES 10
const uint8_t ID3v2_MAGIC[] = {'I', 'D', '3'};

#define ALWAYS_INLINE __attribute__((always_inline)) inline

// Implements a state machine as per Section 10.5 of the datasheet of the VS1053.
//
// These states are usually traversed sequentially from one to the next in order,
// except as stated, and then finally from `PSTEP_WAITING` back to `PSTEP_PLAYING`.
// (The desire for a semi-chonological ordering is why we have not grouped the very
// functionally similar states `PSTEP_FINISHING_EOF` and `PSTEP_FINISHING_CANCEL`
// together.)
typedef enum player_step {
    // There are more bytes to send in the current file.
    PSTEP_PLAYING,
    // We have reached EOF of the current file, and are now
    // sending 2052 bytes of `endFillByte`.
    PSTEP_FINISHING_EOF,
    // We have set `SM_CANCEL`, and are contiuing to send 32 byte chunks of either:
    // 1. (if we were playing:) `endFillByte`, or
    // 2. (if we told to cancel:) the remaining data from the file.
    // In either case we poll `SM_CANCEL` after each chunk to see if
    // it has been cleared yet. If we have sent 2048 bytes, or if we
    // are cancelling and 1 second has elapsed, give up and soft reset
    // the device.
    //
    // If an EOF occurs during this step we simply transition directly to
    // `PSTEP_FINISHING_EOF` which will gracefully handle the situation.
    PSTEP_PENDING_CANCEL_ACK,
    // Same as `PSTEP_FINISHING_EOF`, except occuring due to an explicit cancel.
    PSTEP_FINISHING_CANCEL,
    // We are done sending data, now check that HDAT0 and HDAT1 are both zero.
    // If they aren't, log an error and soft reset the device. We are now waiting
    // for a new file to play.
    PSTEP_WAITING,
} player_step_t;

typedef struct vs1053_player {
    // *** Read-only part ***
    struct player_ctrl {
        vs1053_handle_t dev;

        // A copy of this pin number must be stored here for the ISR to access.
        gpio_num_t pin_dreq;

        // Used (by the player task only) to wait for a signal from the
        // DREQ ISR.
        SemaphoreHandle_t dreq_sem;

        // Used (by the player task only) to wait for when the `player_state`
        // member is updated by any other task, indicating e.g. that a file
        // has been selected to be played.
        SemaphoreHandle_t state_update_sem;

        // Used to protect the `player_state` member, below.
        SemaphoreHandle_t state_mutex;

        // Used to wait the status of the player (e.g. waiting/playing)
        EventGroupHandle_t public_status;
#define PSTATUS_IS_WAITING (1 << 0)
#define PSTATUS_IS_PLAYING (1 << 1)
    } ctrl;

    // *** Read-write part, protected by `player_ctrl.state_mutex` ***
    struct player_state {
#define PLAYER_FD_NOTHING (-1)
#define PLAYER_FD_CANCEL (-2)

        // Stores the file to play when the player task next enters the
        // `PSTEP_WAITING` state. When the player task next transitions
        // from `PSTEP_WAITING` to `PSTEP_PLAYING`, this value is reset
        // to `PLAYER_FD_NOTHING`.
        //
        // Setting the value to anything other than `PLAYER_FD_NOTHING`
        // when the player is in the `PSTEP_PLAYING` step will cause the
        // player to begin a cancel when this field is next read.
        //
        // Acceptable values this field may be set to are (nonnegative)
        // valid fd numbers, `PLAYER_FD_NOTHING`, and `PLAYER_FD_CANCEL`.
        // The value `PLAYER_FD_CANCEL` signals to perform a cancel of the
        // currently playing file, with no new file to transition to
        // afterward.
        int next_fd;

        // Set to a nonzero value when `next_fd` is updated by a task other
        // than the player, and is "given" when the player processes the
        // updated value of `next_fd`. Set to NULL if the updating task
        // does not want a notification. (Only one notification is given,
        // and the value of `processed_next_fd_notify_sem` is set to NULL
        // after the semaphore is given---it is the responsibility of the
        // waiting task to free the semaphore.)
        SemaphoreHandle_t processed_next_fd_notify_sem;

        // This member is initially set to `NULL`. In order to signal the
        // player task to terminate it is set to a nonnull value, and then
        // once shutdown of the player task has progressed sufficiently
        // that it can be guarenteed that no further physical IO with the
        // VS1053 will be peformed, this semaphore is "given". It is then
        // the responsibility of the task waiting on this semaphore to
        // destory it.
        //
        // Note that the job of this semaphore cannot easily be shared with
        // the `processed_next_fd_notify_sem` member (and e.g. with this
        // member becoming a boolean) because of some state-related complexity:
        // we only check whether to stop in the `PSTEP_WAITING` state,
        // so setting `should_stop` while in that state works fine. However
        // if we are in the playing state then `processed_next_fd_notify_sem`
        // will be given as soon as the signal to cancel is recieved, but it
        // would not yet be safe to claim that the player has been destroyed
        // because further IO with the VS1053 will occur during the cancellation
        // proceedure.
        SemaphoreHandle_t should_stop;
    } state;
} vs1053_player_t;

typedef struct player_ctrl player_ctrl_t;
typedef struct player_state player_state_t;

static vs1053_player_t* alloc_player(vs1053_handle_t dev) {
    player_ctrl_t ctrl = {
        .dev = dev,
        .pin_dreq = dev->pin_dreq,
        .dreq_sem = xSemaphoreCreateBinary(),
        .state_update_sem = xSemaphoreCreateBinary(),
        .state_mutex = xSemaphoreCreateMutex(),
        .public_status = xEventGroupCreate(),
    };

    player_state_t state = {
        .next_fd = PLAYER_FD_NOTHING,
        .processed_next_fd_notify_sem = NULL,
        .should_stop = NULL,
    };

    vs1053_player_t* player = malloc(sizeof(vs1053_player_t));
    player->ctrl = ctrl;
    player->state = state;

    return player;
}

static void free_player(vs1053_player_t* player) {
    vSemaphoreDelete(player->ctrl.dreq_sem);
    vSemaphoreDelete(player->ctrl.state_update_sem);
    vSemaphoreDelete(player->ctrl.state_mutex);
    vEventGroupDelete(player->ctrl.public_status);

    // Note that at this point it is safe to access fields
    // of the `player->state` without the protection mutex.
    assert(player->state.next_fd == PLAYER_FD_NOTHING);
    assert(player->state.processed_next_fd_notify_sem == NULL);
    // Note that we do not free `player->state.should_stop`,
    // since that is the responsibility of the waiting task
    // (and it might not have woken yet).

    free(player);
}

static void isr_dreq(void* arg) {
    vs1053_player_t* player = arg;

    // First, since this interrupt is level-triggered (so we don't miss any) disable further
    // interrupts. If the `player_loop()` happens to enable this interrupt again asynchronously
    // with this, we don't mind.
    gpio_intr_disable(player->ctrl.pin_dreq);

    // Only now give the semaphore, which will ensure that the `player_loop()` will always
    // eventually take the semaphore and call `gpio_intr_enable()` for us. (We don't care
    // about the return value, i.e. whether we were able to.)
    xSemaphoreGiveFromISR(player->ctrl.dreq_sem, NULL);
}

static ALWAYS_INLINE void sleep_until_dreq_high(const player_ctrl_t* ctrl) {
    // Clear a potential spurious message from the DREQ ISR.
    xSemaphoreTake(ctrl->dreq_sem, 0);

    // If we are here, DREQ is low. First, enable the DREQ ISR. Since the ISR disables
    // itself before it is possible to take `player->dreq_sem`, a race condition cannot
    // occur here.
    gpio_intr_enable(ctrl->pin_dreq);

    // Wait until the ISR tells us DREQ has gone high again.
    while (xSemaphoreTake(ctrl->dreq_sem, portMAX_DELAY) == pdFALSE)
        ;

    // The concurrency "invariant" at this point is that we are guarenteed that the ISR
    // handler is disabled.
}

// Data structure at the core of the state machine, which is manipulated
// in each iteration.
typedef struct pump_state {
    // Read-only members
    const struct {
        player_ctrl_t ctrl;
    };

    // Read-write members
    struct {
        int fd;
        uint8_t* buff;
        // Used to keep track of the number outstanding valid bytes read into
        // `buff`, but which were not immediately sent because DREQ was low.
        ssize_t bytes_read;
    };
} pump_state_t;

typedef enum pump_stop_reason {
    // Usual termination
    PUMP_STOP_CHUNKS_COMPLETED_OR_DREQ_LOW,

    // Special conditions
    PUMP_STOP_EOF,
    PUMP_STOP_THRESHOLD_REACHED,

    // Unrecoverable error condition
    PUMP_STOP_IO_ERR,
} pump_stop_reason_t;

// If `chunk_iters` is negative, then there is no chunk limit---we only return on some sort of failure
// (even benign, such as EOF), or (more likely) if DREQ is low.
static ALWAYS_INLINE pump_stop_reason_t pump_bytes_until_threshold(pump_state_t* pump, int32_t chunk_iters, ssize_t* out_inc_bytes_transmitted, ssize_t bytes_transmitted_threshold) {
    for (int32_t i = 0; chunk_iters < 0 || i < chunk_iters; i++) {
        if (!pump->bytes_read) {
            if (pump->fd < 0) {
                // If `pump->fd < 0` then we are guarenteed that `pump->buff` has been filled with
                // `endFillByte`, and we have been asked to send chunks of chunks `endFillByte`.
                pump->bytes_read = DATA_CHUNK_SIZE;
            } else {
                pump->bytes_read = read(pump->fd, pump->buff, DATA_CHUNK_SIZE);
            }

            if (pump->bytes_read == 0) {
                ESP_LOGD(TAG, "EOF reached, playback stopped");
                return PUMP_STOP_EOF;
            } else if (pump->bytes_read < 0) {
                ESP_LOGE(TAG, "read failed: 0x%X 0x%X", pump->bytes_read, errno);
                return PUMP_STOP_IO_ERR;
            }
        }

        if (gpio_get_level(pump->ctrl.pin_dreq) == 0) {
            sleep_until_dreq_high(&pump->ctrl);
            return PUMP_STOP_CHUNKS_COMPLETED_OR_DREQ_LOW;
        }

        vs1053_data_transmit(pump->ctrl.dev, pump->buff, pump->bytes_read);
        if (out_inc_bytes_transmitted) {
            *out_inc_bytes_transmitted += pump->bytes_read;
        }
        pump->bytes_read = 0;

        if (bytes_transmitted_threshold >= 0 && *out_inc_bytes_transmitted >= bytes_transmitted_threshold) {
            return PUMP_STOP_THRESHOLD_REACHED;
        }
    }

    return PUMP_STOP_CHUNKS_COMPLETED_OR_DREQ_LOW;
}

// If `chunk_iters` is negative, then there is no chunk limit---we only return on some sort of failure
// (even benign, such as EOF), or (more likely) if DREQ is low.
static ALWAYS_INLINE pump_stop_reason_t pump_bytes(pump_state_t* pump, int32_t chunk_iters) {
    return pump_bytes_until_threshold(pump, chunk_iters, NULL, -1);
}

static ALWAYS_INLINE void check_codec_not_running_or_softreset(vs1053_handle_t dev) {
    uint16_t hdat0 = vs1053_sci_read(dev, VS1053_SCI_HDAT0);
    uint16_t hdat1 = vs1053_sci_read(dev, VS1053_SCI_HDAT1);

    if (hdat0 || hdat1) {
        ESP_LOGW(TAG, "bad hdat values, resetting! (0x%X, 0x%X)", hdat0, hdat1);
        vs1053_reconfigure_to_defaults(dev);
    }
}

static ALWAYS_INLINE void pump_close_fd(pump_state_t* pump) {
    assert(pump->fd >= 0);

    ESP_LOGD(TAG, "closing fd: %d", pump->fd);
    if (close(pump->fd)) {
        ESP_LOGE(TAG, "close failed! (0x%X)", errno);
    }
    pump->fd = -1;
}

static ALWAYS_INLINE void pump_close_fd_and_read_endfill(pump_state_t* pump) {
    pump_close_fd(pump);

    vs1053_sci_write(pump->ctrl.dev, VS1053_SCI_WRAMADDR, VS1053_WRAM_ENDFILLBYTE);
    // Lower 8-bits contain `endFillByte`.
    uint8_t endFillByte = (uint8_t) vs1053_sci_read(pump->ctrl.dev, VS1053_SCI_HDAT1);
    memset(pump->buff, endFillByte, DATA_CHUNK_SIZE);

    ESP_LOGD(TAG, "endFillByte=0x%02X", endFillByte);
}

// DANGER: must be called holding `player->ctrl.state_mutex`!
static void notify_any_waiter_on_next_fd(vs1053_player_t* player) {
    SemaphoreHandle_t notify_sem = player->state.processed_next_fd_notify_sem;
    player->state.processed_next_fd_notify_sem = NULL;

    if (notify_sem) {
        xSemaphoreGive(notify_sem);
    }

    // Note that it is the job of the waiting task to free `notify_sem`.
}

static void player_loop(vs1053_player_t* player) {
    pump_state_t pump = {
        .ctrl = player->ctrl,

        .fd = -1,
        .buff = heap_caps_malloc(DATA_CHUNK_SIZE, MALLOC_CAP_DMA),
        .bytes_read = 0,
    };

    // Number of bytes transmitted while in an `PSTEP_PENDING_CANCEL_ACK` state,
    // as per datasheet we abort if `SM_CANCEL` has not been cleared after
    // 2048 bytes, or if we are cancelling, after one second.
    ssize_t pending_bytes_sent = 0;

    // Number of bytes transmitted while in `PSTEP_FINISHING_*` state,
    // as per datasheet we are required to send at least 2052 (not typo).
    ssize_t finishing_end_fill_bytes_sent = 0;

    player_step_t step = PSTEP_WAITING;
    SemaphoreHandle_t should_stop = NULL;
    xEventGroupSetBits(pump.ctrl.public_status, PSTATUS_IS_WAITING);

    ESP_LOGD(TAG, "loop starting");
    ESP_LOGD(TAG, "-> PSTEP_WAITING");

    while (1) {
        switch (step) {
            case PSTEP_WAITING: {
                // Verify that the last file we were working with was closed.
                assert(pump.fd == -1);
                // Verify that the counters have been reset.
                assert(!pending_bytes_sent);
                assert(!finishing_end_fill_bytes_sent);

                while (xSemaphoreTake(pump.ctrl.state_mutex, portMAX_DELAY) == pdFALSE)
                    ;

                // With `state_mutex` held, clear `state_update_sem`.
                xSemaphoreTake(pump.ctrl.state_update_sem, 0);

                pump.fd = player->state.next_fd;
                player->state.next_fd = PLAYER_FD_NOTHING;
                should_stop = player->state.should_stop;

                notify_any_waiter_on_next_fd(player);

                xSemaphoreGive(pump.ctrl.state_mutex);

                // Note that if `should_stop` was issued to us there still may be a
                // task waiting on `pump.ctrl.state_update_sem`, so we are sure to
                // only enter the "stop path" after dispatching any such pending
                // notifications. Similarly if `player->state.next_fd` as nonnegative
                // then the fd will be closed in the "stop path".
                if (should_stop) {
                    goto player_loop_stop;
                }

                if (pump.fd >= 0) {
                    goto player_loop_transition_to_playing;
                }

                // Wait for another update to `player->state.next_fd`.
                while (xSemaphoreTake(pump.ctrl.state_update_sem, portMAX_DELAY) == pdFALSE)
                    ;

                assert(pump.fd < 0);
                pump.fd = -1;

                break;
            }
            case PSTEP_PLAYING: {
                while (xSemaphoreTake(pump.ctrl.state_mutex, portMAX_DELAY) == pdFALSE)
                    ;

                int next_fd = player->state.next_fd;

                xSemaphoreGive(pump.ctrl.state_mutex);

                // Check if we need to cancel
                if (next_fd != PLAYER_FD_NOTHING) {
                    goto player_loop_transition_to_pending_cancel_ack;
                }

                // If we are here, we don't need to cancel, and can continue playing the file.
                pump_stop_reason_t reason = pump_bytes(&pump, PLAYER_CHUNK_ITERS);
                switch (reason) {
                    case PUMP_STOP_CHUNKS_COMPLETED_OR_DREQ_LOW: {
                        // This is the normal reason for stopping the byte pump. Sleeping until
                        // DREQ is high (if this was neccesary) is peformed by `pump_bytes()`
                        // itself before it returns.
                        break;
                    }
                    case PUMP_STOP_EOF: {
                        // We have reached the EOF (end of file). Proceed to the EOF handling
                        // procedure as per datasheet.
                        goto player_loop_transition_to_finishing_eof;
                    }
                    default: {
                        ESP_LOGE(TAG, "unrecoverable error: 0x%X", reason);
                        goto player_loop_unrecoverable_error;
                    }
                }

                break;
            }
            case PSTEP_PENDING_CANCEL_ACK: {
                // Note that if `fd >= 0` then the cancel was caused by an explicit request
                // to cancel playback. If `fd == -1` then the cancel was caused as part of the
                // graceful end-of-file sequence as specified in the datasheet (10.5.1).

                // In this step we pump 32 byte chunks one-at-a-time, checking `VS1053_SM_CANCEL`
                // each time as we go.
                pump_stop_reason_t reason = pump_bytes_until_threshold(&pump, 1, &pending_bytes_sent, PENDING_CANCEL_GIVE_UP_BYTES);
                switch (reason) {
                    case PUMP_STOP_CHUNKS_COMPLETED_OR_DREQ_LOW: {
                        // This is the normal reason for stopping the byte pump. Sleeping until
                        // DREQ is high (if this was neccesary) is peformed by `pump_bytes()`
                        // itself before it returns.
                        break;
                    }
                    case PUMP_STOP_EOF: {
                        // This is an unfortunate edge-case---when we are trying to perform an explicit
                        // cancel, part of which involves continuing to send valid data from the file for
                        // for a time, and we happen to also run into EOF. In this case we proceed direct
                        // to the standard EOF handler, trusting that then the situation will gracefully
                        // be resolved (the datasheet says nothing regarding this specific case).
                        if (pump.fd >= 0) {
                            pending_bytes_sent = 0;
                            goto player_loop_transition_to_finishing_eof;
                        }

                        // If `pump.fd == -1`, this should never occur, so fail.
                    }
                    // FALLTHROUGH
                    case PUMP_STOP_THRESHOLD_REACHED: {
                        // Unfortunately we have sent `PENDING_CANCEL_GIVE_UP_BYTES` bytes
                        // and the cancel has still not been acknowledged; give up and soft
                        // reset the device.
                        if (pump.fd >= 0) {
                            pump_close_fd(&pump);
                        }

                        ESP_LOGW(TAG, "cancel lockup detected, resetting!");

                        vs1053_reconfigure_to_defaults(pump.ctrl.dev);

                        pending_bytes_sent = 0;
                        goto player_loop_transition_to_waiting;
                    }
                    default: {
                        ESP_LOGE(TAG, "unrecoverable error: 0x%X", reason);
                        goto player_loop_unrecoverable_error;
                    }
                }

                // Check whether `VS1053_SM_CANCEL` has been cleared yet.
                if (!(vs1053_sci_read(pump.ctrl.dev, VS1053_SCI_MODE) & VS1053_SM_CANCEL)) {
                    // If `VS1053_SM_CANCEL` has been cleared, we could have entered this step
                    // because of either an EOF on file read (end of normal playing) or a
                    // requested cancellation.
                    pending_bytes_sent = 0;

                    if (pump.fd < 0) {
                        // If it was due to a genuine EOF, we are done and can start over.
                        goto player_loop_transition_to_waiting;
                    }

                    goto player_loop_transition_to_finishing_cancel;
                }

                // TODO As per datasheet, if we are in this step due to an explicit cancel
                // (i.e. `pump.fd != -1`), then soft reset if we have been in this step for
                // more than 1 second.

                break;
            }
            case PSTEP_FINISHING_EOF:
            case PSTEP_FINISHING_CANCEL: {
                assert(pump.fd == -1);

                // Send at least `FINISHING_ENDFILL_BYTES` bytes of `endFillByte`, which is guarenteed
                // by this point to be read into `pump.buff`.
                pump_stop_reason_t reason = pump_bytes_until_threshold(&pump, -1, &finishing_end_fill_bytes_sent, FINISHING_ENDFILL_BYTES);
                switch (reason) {
                    case PUMP_STOP_CHUNKS_COMPLETED_OR_DREQ_LOW: {
                        // There are more bytes to send; let's go again.
                        break;
                    }
                    case PUMP_STOP_THRESHOLD_REACHED: {
                        // We have sent at least `FINISHING_ENDFILL_BYTES` bytes of `endFillByte`,
                        // so it is time to transition to the next step.
                        finishing_end_fill_bytes_sent = 0;

                        switch (step) {
                            case PSTEP_FINISHING_EOF: {
                                goto player_loop_transition_to_pending_cancel_ack;
                            }
                            case PSTEP_FINISHING_CANCEL: {
                                goto player_loop_transition_to_waiting;
                            }
                            default: {
                                abort();
                            }
                        }
                        break;
                    }
                    default: {
                        ESP_LOGE(TAG, "unrecoverable error: 0x%X", reason);
                        goto player_loop_unrecoverable_error;
                    }
                }
            }
        }

        continue;

    player_loop_transition_to_playing : {
        // As per datasheet, write zero to the decode time register twice
        // in order to be sure that we reset it.
        vs1053_sci_write(pump.ctrl.dev, VS1053_SCI_DECODE_TIME, 0x0000);
        vs1053_sci_write(pump.ctrl.dev, VS1053_SCI_DECODE_TIME, 0x0000);

        step = PSTEP_PLAYING;
        xEventGroupClearBits(pump.ctrl.public_status, PSTATUS_IS_WAITING);
        xEventGroupSetBits(pump.ctrl.public_status, PSTATUS_IS_PLAYING);
        ESP_LOGD(TAG, "-> PSTEP_PLAYING");
        continue;
    }

    player_loop_transition_to_waiting : {
        // As per the datasheet, in this case we are now done and may return to
        // the `PSTEP_WAITING` state.
        check_codec_not_running_or_softreset(pump.ctrl.dev);
        step = PSTEP_WAITING;
        xEventGroupSetBits(pump.ctrl.public_status, PSTATUS_IS_WAITING);
        ESP_LOGD(TAG, "-> PSTEP_WAITING");
        continue;
    }

    player_loop_transition_to_finishing_eof : {
        // In the next step we will be sending `endFillByte`: it is time to close
        // `pump.fd` and read `endFillByte` into `buff`.
        pump_close_fd_and_read_endfill(&pump);
        step = PSTEP_FINISHING_EOF;
        xEventGroupClearBits(pump.ctrl.public_status, PSTATUS_IS_PLAYING);
        ESP_LOGD(TAG, "-> PSTEP_FINISHING_EOF");
        continue;
    }

    player_loop_transition_to_finishing_cancel : {
        // In the next step we will be sending `endFillByte`: it is time to close
        // `pump.fd` and read `endFillByte` into `buff`.
        pump_close_fd_and_read_endfill(&pump);
        step = PSTEP_FINISHING_CANCEL;
        ESP_LOGD(TAG, "-> PSTEP_FINISHING_CANCEL");
        continue;
    }

    player_loop_transition_to_pending_cancel_ack : {
        vs1053_sci_write(pump.ctrl.dev, VS1053_SCI_MODE,
                         vs1053_sci_read(pump.ctrl.dev, VS1053_SCI_MODE) | VS1053_SM_CANCEL);
        // As per the datasheet, keep sending bytes and polling the cancel bit.
        step = PSTEP_PENDING_CANCEL_ACK;
        xEventGroupClearBits(pump.ctrl.public_status, PSTATUS_IS_PLAYING);
        ESP_LOGD(TAG, "-> PSTEP_PENDING_CANCEL_ACK");
        continue;
    }
    }

    abort();

player_loop_stop : {
    ESP_LOGD(TAG, "loop stopping");
    xSemaphoreGive(should_stop);

    assert(pump.fd == PLAYER_FD_CANCEL);

    while (xSemaphoreTake(pump.ctrl.state_mutex, portMAX_DELAY) == pdFALSE)
        ;

    // Assert that any pending operation has been cleaned up
    // already.
    assert(player->state.next_fd == PLAYER_FD_NOTHING);
    assert(player->state.processed_next_fd_notify_sem == NULL);
    assert(player->state.should_stop);

    xSemaphoreGive(pump.ctrl.state_mutex);

    free(pump.buff);

    return;
}

player_loop_unrecoverable_error : {
    free(pump.buff);

    // TODO Report IO errors more gracefully
    abort();
}
}

// DANGER: This function must be run on a task which is pinned to a core.
static void player_task(void* arg) {
    vs1053_player_t* player = arg;

    ESP_LOGD(TAG, "player task started");

    gpio_num_t pin_dreq = player->ctrl.pin_dreq;

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_HIGH_LEVEL,
        .pin_bit_mask = (1ULL << pin_dreq),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Prevent the level trigger interrupt on the DREQ pin
    // from hanging up the core when we enable interrupts.
    gpio_intr_disable(pin_dreq);

    // TODO Install in a way which doesn't collide with other code.
    // (Note that we need `gpio_install_isr_service()` to have been
    // called without the flag which requires IRAM ISRs.)
    gpio_install_isr_service(0);
    gpio_isr_handler_add(pin_dreq, isr_dreq, arg);

    player_loop(player);

    if (xSemaphoreTake(player->ctrl.state_mutex, 0) == pdFALSE) {
        ESP_LOGE(TAG, "cannot take `state_mutex` after `player_loop()` exit");
        abort();
    }

    if (!player->state.should_stop) {
        ESP_LOGE(TAG, "`player_loop()` exited without stop!");
        abort();
    }

    UBaseType_t stack_remaining = uxTaskGetStackHighWaterMark(NULL);
    if (stack_remaining <= 0) {
        ESP_LOGE(TAG, "stack overflow detected!");
    }

    ESP_LOGD(TAG, "stack words remaining: %u", stack_remaining);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_isr_handler_remove(player->ctrl.pin_dreq);
    gpio_uninstall_isr_service();

    gpio_intr_enable(player->ctrl.pin_dreq);

    free_player(player);

    ESP_LOGD(TAG, "player task stopped");

    vTaskDelete(NULL);
}

static void update_player_state_maybe_waiting(vs1053_player_t* player, int next_fd, SemaphoreHandle_t should_stop, bool wait_until_processed) {
    SemaphoreHandle_t notify_sem = wait_until_processed ? xSemaphoreCreateBinary() : NULL;

    // Take the state mutex.
    while (xSemaphoreTake(player->ctrl.state_mutex, portMAX_DELAY) == pdFALSE)
        ;

    assert(player->state.next_fd != next_fd);

    // If we are overwritting a nonnegative value, i.e. an honest fd number,
    // then we must free it. If a task is waiting on the processing of the
    // `next_fd` we are overwriting, then wake it.
    if (player->state.next_fd >= 0) {
        close(player->state.next_fd);
        notify_any_waiter_on_next_fd(player);
    }

    // The `should_stop` signal can only be sent once, and no further
    // updates are then permitted. (Because allowing such would both
    // be useless in practice and also would create a race condition.)
    assert(!player->state.should_stop);

    player->state.next_fd = next_fd;
    player->state.processed_next_fd_notify_sem = notify_sem;
    if (should_stop) {
        player->state.should_stop = should_stop;
    }

    // Give the state mutex.
    xSemaphoreGive(player->ctrl.state_mutex);

    // Wake the player task.
    xSemaphoreGive(player->ctrl.state_update_sem);

    if (notify_sem) {
        // Wait for the value of `next_fd` to be processed by the player. Note that
        // we can never give up here, since it is our job to free `notify_sem` after
        // we are woken using it.
        while (xSemaphoreTake(notify_sem, portMAX_DELAY) == pdFALSE)
            ;

        vSemaphoreDelete(notify_sem);
    }
}

static esp_err_t start_playing_fd_maybe_waiting(vs1053_player_t* player, int fd, bool wait_until_processed) {
    uint8_t* buff = heap_caps_malloc(ID3v2_HEADER_BYTES, MALLOC_CAP_DMA);
    int bytes_read = read(fd, buff, ID3v2_HEADER_BYTES);
    if (bytes_read != ID3v2_HEADER_BYTES) {
        ESP_LOGE(TAG, "read failed: 0x%X (0x%X)", bytes_read, errno);

        free(buff);
        goto start_playing_fail_close_fd;
    }

    // If the file begins with an (MP3) ID3v2 header, skip past it.
    uint32_t seek_pos = 0;
    if (!memcmp(buff, ID3v2_MAGIC, sizeof(ID3v2_MAGIC))) {
        uint32_t size = 0;
        for (int i = 0; i < 4; i++) {
            size <<= 7;
            size |= buff[i + 6] & 0x7F;
        }

        ESP_LOGD(TAG, "MP3 ID3v2 header detected, file offset at: 0x%X", size);

        seek_pos = size;
    }

    free(buff);

    if (lseek(fd, seek_pos, SEEK_SET) != seek_pos) {
        ESP_LOGE(TAG, "could not seek! (0x%X)", errno);
        return ESP_FAIL;
    }

    // It is now time to actually inform the player task that a file is waiting to
    // be played. (Thus we acquire the neccesary lock.)
    update_player_state_maybe_waiting(player, fd, false, wait_until_processed);

    return ESP_OK;

start_playing_fail_close_fd:
    if (close(fd)) {
        ESP_LOGE(TAG, "close failed! (0x%X)", errno);
    }

    return ESP_FAIL;
}

static esp_err_t start_playing_file_maybe_waiting(vs1053_player_t* player, const char* path, bool wait_until_processed) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "could not open file! '%s' (0x%X)", path, errno);
        return ESP_FAIL;
    }

    return start_playing_fd_maybe_waiting(player, fd, wait_until_processed);
}

esp_err_t vs1053_player_start_playing_fd(vs1053_player_t* player, int fd) {
    ESP_LOGD(TAG, "playing fd: %d", fd);
    return start_playing_fd_maybe_waiting(player, fd, true);
}

esp_err_t vs1053_player_start_playing_fd_nowait(vs1053_player_t* player, int fd) {
    ESP_LOGD(TAG, "playing fd (nowait): %d", fd);
    return start_playing_fd_maybe_waiting(player, fd, false);
}

esp_err_t vs1053_player_start_playing_file(vs1053_player_t* player, const char* path) {
    ESP_LOGD(TAG, "playing: %s", path);
    return start_playing_file_maybe_waiting(player, path, true);
}

esp_err_t vs1053_player_start_playing_file_nowait(vs1053_player_t* player, const char* path) {
    ESP_LOGD(TAG, "playing (nowait): %s", path);
    return start_playing_file_maybe_waiting(player, path, false);
}

void vs1053_player_cancel(vs1053_player_t* player) {
    ESP_LOGD(TAG, "cancel playing");
    update_player_state_maybe_waiting(player, PLAYER_FD_CANCEL, false, true);
}

void vs1053_player_cancel_nowait(vs1053_player_t* player) {
    ESP_LOGD(TAG, "cancel playing (nowait)");
    update_player_state_maybe_waiting(player, PLAYER_FD_CANCEL, false, false);
}

// Wait until the last file has finished being played/cancelled, or `timeout_millis`,
// whichever is sooner. A negative value of `timeout_millis` represents an infinite timeout.
//
// Returns `true` if the player finished, `false` if the timeout was reached.
bool vs1053_player_sleep_until_player_waiting(vs1053_player_t* player, int32_t timeout_millis) {
    bool is_waiting;

    do {
        EventBits_t bits = xEventGroupWaitBits(player->ctrl.public_status, PSTATUS_IS_WAITING, pdFALSE, pdTRUE,
                                               timeout_millis < 0 ? portMAX_DELAY : (timeout_millis / portTICK_PERIOD_MS));
        is_waiting = bits & PSTATUS_IS_WAITING;
        // If `timeout_millis < 0`, never give up on timeout.
    } while (timeout_millis < 0 && !is_waiting);

    return is_waiting;
}

bool vs1053_player_is_playing(vs1053_player_t* player) {
    return xEventGroupGetBits(player->ctrl.public_status) & PSTATUS_IS_PLAYING;
}

bool vs1053_player_is_waiting(vs1053_player_t* player) {
    return xEventGroupGetBits(player->ctrl.public_status) & PSTATUS_IS_WAITING;
}

esp_err_t vs1053_player_create(vs1053_handle_t dev, vs1053_player_handle_t* out_player) {
    // Reset and configure the VS1053, setting clock speed, etc.
    esp_err_t ret = vs1053_reconfigure_to_defaults(dev);
    if (ret != ESP_OK) {
        return ret;
    }

    vs1053_player_t* player = alloc_player(dev);

    // Note that this task will allocate an interrupt, and therefore must happen on a well-defined core
    // (hence `xTaskCreatePinnedToCore()`), since interrupts must be deallocated on the same core they
    // were allocated on.
    BaseType_t result = xTaskCreatePinnedToCore(&player_task, "vs1053_player", TASK_STACK_SIZE, (void*) player,
                                                10, NULL, RUN_ON_CORE_NUM);
    if (result != pdPASS) {
        free_player(player);

        ESP_LOGE(TAG, "failed to create player task! (0x%X)", result);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "created player: %p", player);

    *out_player = player;
    return ESP_OK;
}

void vs1053_player_destroy(vs1053_player_handle_t player) {
    ESP_LOGD(TAG, "destroying player: %p", player);

    // Because the player task keeps a pointer to itself, at this point all we need to do is
    // to signal to the player task to stop, and wait for that to be acknowledged. (When the
    // acknowledgement happens neccesarily the player task will be in the `PSTEP_WAITING` state,
    // which in turn means that no further communication with the physical VS1053 hardware will
    // occur).
    SemaphoreHandle_t should_stop = xSemaphoreCreateBinary();

    update_player_state_maybe_waiting(player, PLAYER_FD_CANCEL, should_stop, true);

    // Wait for the signal that stopping has sufficiently completed.
    while (xSemaphoreTake(should_stop, portMAX_DELAY) == pdFALSE)
        ;

    // Free the signally semaphore.
    vSemaphoreDelete(should_stop);
}