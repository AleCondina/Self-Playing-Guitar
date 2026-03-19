/**
 ******************************************************************************
 * @file    song_player.h
 * @brief   Event-driven song player with look-ahead motor positioning
 *
 * Replaces the fixed-delay solenoid actuation in TuningCtrl_RunHop() with
 * a state machine that guarantees every note fires within 0.2 Hz of target.
 *
 * Note lifecycle:
 *   PREPARE  -> CONVERGE -> FIRE -> SUSTAIN -> [next PREPARE or CONVERGE]
 *
 * Key features:
 *   - Motors pre-positioned during previous note's sustain (look-ahead)
 *   - PI converges BEFORE solenoid fires (clean FFT, no EMI)
 *   - FFT blanked during solenoid bursts to prevent false PI corrections
 *   - Chord strings fire simultaneously via batch solenoid commands
 *
 * Usage:
 *   SongPlayer_Init();              // once at startup (optional, statics zero-init)
 *   SongPlayer_Start(song_id);      // begin playback
 *   SongPlayer_Hop();               // call every FFT hop (~170ms)
 *   SongPlayer_Stop();              // stop playback
 ******************************************************************************
 */

#ifndef SONG_PLAYER_H
#define SONG_PLAYER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PLAYER_IDLE,       /**< No song playing                                  */
    PLAYER_PREPARE,    /**< Motors moving to LUT positions + cold-start burst */
    PLAYER_CONVERGE,   /**< PI refining (FFT clean, no solenoid active)      */
    PLAYER_FIRE,       /**< Solenoid fires (single-hop transition)           */
    PLAYER_SUSTAIN,    /**< Note ringing, FFT blanked, look-ahead active     */
} PlayerPhase_t;

/** Initialise player state to safe defaults. */
void SongPlayer_Init(void);

/** Start playback of the given song. */
void SongPlayer_Start(uint8_t song_id);

/** Stop playback and disable motors. */
void SongPlayer_Stop(void);

/** Run one state-machine hop.  Call every FFT hop (~170 ms). */
void SongPlayer_Hop(void);

/** Return the current phase of the state machine. */
PlayerPhase_t SongPlayer_GetPhase(void);

/** Return 1 if a song is currently playing, 0 otherwise. */
uint8_t SongPlayer_IsPlaying(void);

/** Notes fired without full PI lock (diagnostic). */
uint32_t SongPlayer_GetTimeoutCount(void);

/** Notes fired with full PI lock (diagnostic). */
uint32_t SongPlayer_GetLockedCount(void);

#ifdef __cplusplus
}
#endif

#endif /* SONG_PLAYER_H */
