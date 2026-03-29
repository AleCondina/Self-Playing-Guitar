# Smoke1 (Song1) Performance Fix — Changelog

## Problem

Song1 (Smoke on the Water, single E2 string) had three critical issues:

1. **First note not actuating** — the solenoid fires too late, and the note expires before the controller is ready
2. **Song too fast / notes missing** — PI gets almost no time to correct before the scheduler advances
3. **Completely out of tune** — feedforward lands in the wrong position and PI never gets a chance to fix it

## Root Cause

**Timing race between the song scheduler and the tuning controller pipeline.**

The song scheduler starts counting note duration from its first tick, but the tuning controller pipeline (cold-start deferral, solenoid burst, resonance wait, feedforward move, FFT clear wait) takes ~680ms before PI can engage. For a 1000ms note, that leaves only 1-2 PI hops (~320ms). For the 500ms note (#5, As2), PI gets zero hops.

### Pipeline overhead trace (note 0, E2, 1000ms, burst=72ms)

| Hop | Time   | Song Timer  | Controller State    | What Happens                                                     |
|-----|--------|-------------|---------------------|------------------------------------------------------------------|
| 0   | 0ms    | Clock starts| IDLE                | Cold-start `pending=2` → decrement to 1. No solenoid, no motor. |
| 1   | 171ms  | 171/1000    | RESONANCE_WAIT      | `pending` → 0, solenoid fires. `StartNoteTransition()` called.  |
| 2   | 341ms  | 341/1000    | FF_MOVE → FF_WAIT   | Resonance wait expires. FF target computed. Motor starts moving. |
| 3   | 512ms  | 512/1000    | FF_WAIT             | FFT still blocked (clear_wait=1). Motor possibly still moving.  |
| 4   | 682ms  | 682/1000    | FF_WAIT → PI?       | FFT clear expires. If motor stopped + FFT detects E2 → PI.      |
| 5   | 853ms  | 853/1000    | PI_ACTIVE           | First (possibly only) PI correction hop.                         |
| 6   | 1024ms | **EXPIRED** | —                   | Note advances to #1 (G2). Pipeline restarts.                     |

**Result: PI gets 1-2 hops max per 1000ms note. 500ms notes get zero PI time.**

---

## Changes Made

### Files Modified

| File | Change |
|------|--------|
| `CM7/App/Inc/tuning_controller.h` | Added `last_pi_delta` field to `TuningStringState_t` |
| `CM7/App/Src/tuning_controller.c` | Per-hop logging, note-transition events, note timer sync |
| `CM7/App/Src/song_scheduler.c`    | `SongSched_ResetNoteTimer()`, burst_ms 72→125, test songs |
| `CM7/App/Inc/song_scheduler.h`    | Declared `SongSched_ResetNoteTimer()` |
| `CM7/App/Src/app_cm7.c`           | Enabled `SONG_LOCK_WAIT_MODE = 1u` |

---

### Step 1: Per-hop diagnostic logging

**No behavior change — instrumentation only.**

Replaced the decimated (every 5 hops) `[TC]` CSV with **every-hop** `[SNG]` CSV:

```
[SNG] timestamp_ms, note_idx, hop_in_note, state, tgt_hz, meas_hz, err_semi, ff_steps, pi_delta, cmd_steps, cur_steps, motor_busy, fft_clr_wait, res_wait, lock, mag
```

New fields vs old CSV:
- `note_idx` — which song note is active
- `hop_in_note` — hops since this note started (from `hop_count`)
- `pi_delta` — PI correction output (stored in new `last_pi_delta` field)
- `motor_busy` — CM4 motor_is_moving flag
- `fft_clr_wait` / `res_wait` — remaining wait counters

Added `[NOTE]` transition event logging before each `StartNoteTransition()`:
```
[NOTE] t=1024ms idx→1 str0 tgt=98.00 Hz state=4 hops=5 lock=0 err=0.0234
```

### Step 2: Note timer synchronization

Added `SongSched_ResetNoteTimer(uint32_t now_ms)` to reset `s_note_start_ms` for the current note. Called from the tuning controller right after `StartNoteTransition()` fires.

**Effect:** Note duration now counts from the moment the solenoid actually fires, not from the first scheduler tick. This recovers the ~171ms lost to cold-start deferral on the first note and ensures every note gets its full scheduled duration for PI correction.

### Step 3: Lock-wait mode enabled

Changed `SONG_LOCK_WAIT_MODE` from `0u` to `1u` in `app_cm7.c`.

**Effect:** The song scheduler holds on each note until PI achieves lock (error < 0.05 semitones) before advancing. This guarantees tuning accuracy at the cost of rhythmic timing — notes may last longer than their scheduled duration if the controller is slow to lock.

### Step 4: Increased E2 burst duration

Changed all song1 burst_ms from `72u` to `125u`.

**Rationale:** At E2 (82.41 Hz, period ~12ms), a 72ms burst is only ~6 cycles — marginal for reliable FFT detection. 125ms gives ~10 cycles and a stronger spectral peak. `BurstClearWaitHops` remains 3 for both values (both ceil to the same hop count), so no additional pipeline delay.

### Step 5: Test songs

Added three test songs for incremental validation:

| Song ID | Command  | Description                     | Purpose                              |
|---------|----------|---------------------------------|--------------------------------------|
| 2       | `play 2` | E2 hold for 10s                 | Validate full pipeline end-to-end    |
| 3       | `play 3` | E2 → G2, 5s each               | Validate note transitions            |
| 4       | `play 4` | E2 → G2, 1s each               | Stress test with short note durations|

---

## Test Protocol

### Test 1: Single note hold (`play 2`)
- **Expect:** Solenoid fires, FF moves to E2 position (~-71 steps), PI locks within 2-3 hops, stays locked for 10s
- **Verify in logs:** `[SNG]` shows state progression 0(IDLE) → 1(RESONANCE_WAIT) → 2(FF_MOVE) → 3(FF_WAIT) → 4(PI_ACTIVE) → 5(IN_TUNE)
- **Pass criteria:** `lock=1` within 5 hops of note start, `err_semi < 0.05` sustained

### Test 2: Two-note slow transition (`play 3`)
- **Expect:** E2 locks, after 5s transitions to G2 (98.00 Hz), solenoid fires, FF moves to ~fret 3 position, PI locks
- **Verify:** `[NOTE]` transition event prints with lock=1 for note 0. Second note reaches PI_ACTIVE.
- **Pass criteria:** Both notes achieve lock. Note timer starts from solenoid fire.

### Test 3: Two-note fast transition (`play 4`)
- **Expect:** With lock-wait mode, each note holds until locked even though scheduled for 1s
- **Verify:** Compare `[SNG]` timestamps — actual note duration should exceed 1000ms if lock takes longer
- **Pass criteria:** Both notes achieve lock. Measure actual vs scheduled duration.

### Test 4: Full song1 (`play 1`)
- **Expect:** Every note actuates (solenoid fires), FF gets close, PI corrects residual
- **Verify:** No notes skip PI_ACTIVE entirely. All `[NOTE]` transitions show `lock=1`.
- **Pass criteria:** >80% of notes reach IN_TUNE before advancing

---

## State Machine Reference

| Value | State              | Description                              |
|-------|--------------------|------------------------------------------|
| 0     | TUNE_STATE_IDLE    | No target, waiting                       |
| 1     | RESONANCE_WAIT     | Post-burst breathing before FF           |
| 2     | FF_MOVE            | Feedforward compute + motor command      |
| 3     | FF_WAIT            | Waiting for clean FFT + motor stop       |
| 4     | PI_ACTIVE          | PI feedback running                      |
| 5     | IN_TUNE            | Within deadband, monitoring              |

## Log Format Quick Reference

**Per-hop CSV (`[SNG]`):**
```
[SNG] ts, note_idx, hop, state, tgt_hz, meas_hz, err_semi, ff, pi_d, cmd, cur, mbusy, fft_w, res_w, lock, mag
```

**Note transition event (`[NOTE]`):**
```
[NOTE] t=<ms> idx→<new_idx> str<id> tgt=<new_hz> Hz state=<prev_state> hops=<count> lock=<0|1> err=<semi>
```
