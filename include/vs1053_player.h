#ifndef __LIB_VS1053_PLAYER_H
#define __LIB_VS1053_PLAYER_H

#include <esp_err.h>
#include <stdint.h>

#include "vs1053.h"

typedef struct vs1053_player vs1053_player_t;
typedef vs1053_player_t* vs1053_player_handle_t;

// Creates a player with underlying device the passed `vs1053_handle_t`. Only one player
// for any given fixed `vs1053_handle_t` can exist at particular time (but there may be
// multiple players for multiple devices). Use `vs1053_player_destroy()` to destroy
// a player.
//
// Moreover, it is not allowed to directly interact with the `vs1053_handle_t`
// used to create a player while the player is not guarenteed to be in the "waiting"
// state (this in turn can be assured by calling `vs1053_player_sleep_until_player_waiting()`
// after after all player API commands on any thread have been issued---note that
// polling `vs1053_player_is_waiting()` is not acceptable for this purpose and will
// result in race conditions). Thus for example the sequence:
// ````
//      vs1053_player_cancel(player);
//      vs1053_player_sleep_until_player_waiting(player, -1);
//      vs1053_ctrl_set_volume(dev, left_vol, right_vol);
//      ESP_ERROR_CHECK(vs1053_player_start_playing_file(player, "/sdcard/myfile.mp3"));
// ````
// Would be required in order to stop the currently playing file, change volume,
// and then play a new file. We would also need to be sure in this context that no
// other threads could call any player API function at the same time as this sequence
// was executing. While on the other hand only a single call:
// ````
//      ESP_ERROR_CHECK(vs1053_player_start_playing_file(player, "/sdcard/myfile.mp3"));
// ````
// would be neccesary to change the currently playing file (since cancellation of the
// currently playing file is handled automatically in this case). Moreover, thread safety
// of this function means that in this case any other threads simultaneously calling
// player API functions would be totally acceptable. Note that in either case, it is not
// acceptable for any other threads to interact with the underlying device (using the
// `vs1053_handle_t` used to create the player---only player API functions may be used).
esp_err_t vs1053_player_create(vs1053_handle_t dev, vs1053_player_handle_t* player);

// This function is thread-safe, and may be called when the player is in any state.
//
// When this function returns the player thread may still be in the process
// of cleaning up, but this library guarentees no further communication with
// the VS1053 hardware will occur; therefore the underlying `vs1053_handle_t`
// used to create the player is again safe to use without restriction.
//
// After calling this function it is undefined behaviour to ever call
// any API function again passing the same `vs1053_player_handle_t`.
void vs1053_player_destroy(vs1053_player_handle_t player);

// These functions are thread-safe. However, interacting with the underlying
// `vs1053_handle_t` on any other thread is *not* permitted. See the comment
// on `vs1053_player_create()`.
//
// These functions start the player playing the given file, returning once
// playback has actually started. In particular, it is safe to call
// `vs1053_player_sleep_until_player_waiting()` immediately after a call to
// either of these functions without a race condition (there may be a short
// delay after the player is instructed to play data in order for it to actually
// transition away from the "waiting" state and into the playing state).
//
// Note that by passing `fd` to `vs1053_player_start_playing_fd()` it becomes the
// responsibility of the player to close `fd`, and the caller should never do this.
esp_err_t vs1053_player_start_playing_fd(vs1053_player_handle_t player, int fd);
esp_err_t vs1053_player_start_playing_file(vs1053_player_handle_t player, const char* path);

// Cancel playback of the currently playing file. This function may return before
// the player reports it is no longer in the playing state/before it has reached
// the waiting state.
void vs1053_player_cancel(vs1053_player_handle_t player);

// Verions of functions above with a `*_nowait` suffix. They do not wait for the
// requested operation to complete (which is usually a very short time---note that
// e.g. `vs1053_player_start_playing_fd()`  waits until playing *starts*, not ends).
// To wait for playing to end use `vs1053_player_sleep_until_player_waiting()`.
//
// These functions are still thread safe, but a race condition may occur if these functions
// are paired with `vs1053_player_sleep_until_player_waiting()` (to avoid this, use the
// versions without "nowait" instead).
esp_err_t vs1053_player_start_playing_fd_nowait(vs1053_player_handle_t player, int fd);
esp_err_t vs1053_player_start_playing_file_nowait(vs1053_player_handle_t player, const char* path);
void vs1053_player_cancel_nowait(vs1053_player_handle_t player);

// Wait until the player has entered the "waiting" state, implying that current file has
// finished being played/cancelled, or `timeout_millis`, whichever is sooner. In the "waiting"
// state the player is guarenteed to not perform any SPI transactions to the VS1053, and
// will not exit this state until `vs1053_player_play_*()` is called. The API guarentees that
// calling this function after using `vs1053_player_start_playing_*()` to start playback
// will never result in a race condition: there may be a short delay after the player is
// instructed to play data in order for it to actually transition away from the "waiting"
// state and into the "playing" state, but `vs1053_player_start_playing_*()` will always
// return *after* this has occurred.
//
// A negative value of `timeout_millis` represents an infinite timeout.
// Returns `true` if the player finished, `false` if the timeout was reached.
bool vs1053_player_sleep_until_player_waiting(vs1053_player_handle_t player, int32_t timeout_millis);

// Returns the actual state of the player describing whether it is currently playing
// a file. Note that after a call to `vs1053_player_start_playing_*()` this function may
// return `false` if playback has already finished.
bool vs1053_player_is_playing(vs1053_player_handle_t player);

// Returns whether the player is in the "waiting" state, during which the player
// is guarenteed to not perform any SPI transactions to the VS1053. Note that for short
// periods during state transitions the player may neither report that it is playing
// nor waiting.
bool vs1053_player_is_waiting(vs1053_player_handle_t player);

#endif