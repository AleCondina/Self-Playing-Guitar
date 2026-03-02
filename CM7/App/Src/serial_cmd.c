/**
 ******************************************************************************
 * @file    serial_cmd.c
 * @brief   Serial command interface for motor control
 * @note    Parses text commands from UART and dispatches inter-core calls
 *
 *          Commands:
 *            start                    Start system (after homing)
 *            speed <id> <steps/s>     Set motor speed (neg = reverse)
 *            accel <id> <value>       Set max acceleration
 *            maxspeed <id> <value>    Set max speed limit
 *            move <id> <steps>        Relative move
 *            goto <id> <pos>|<name>   Absolute move or recall preset
 *            microstep <id> <0-5>     Set microstep mode
 *            enable <id>              Enable motor driver
 *            disable <id>             Disable motor driver
 *            zero <id>                Zero position counter
 *            home                     Re-home all motors
 *            save <name> [id]         Save position as preset
 *            presets                  List saved presets
 *            stop                     Emergency stop all
 *            status                   Print motor status
 *            help                     Show command list
 ******************************************************************************
 */

#include "serial_cmd.h"
#include "debug_uart.h"
#include "inter_core_cm7.h"
#include "shared_memory.h"
#include "application_config.h"
#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define CMD_BUFFER_SIZE  128

/* Preset storage */
#define MAX_PRESETS      16
#define PRESET_NAME_LEN  16

typedef struct {
    char name[PRESET_NAME_LEN];
    uint8_t motor_id;
    int32_t position;       /* Position in steps */
    uint8_t used;
} Preset_t;

static char cmd_buffer[CMD_BUFFER_SIZE];
static Preset_t presets[MAX_PRESETS] = {0};
static uint8_t system_started = 0;

/* Microstep multipliers for degrees calculation */
static const uint16_t microstep_multipliers[] = {1, 2, 4, 8, 16, 32};
static const char *microstep_names[] = { "full", "1/2", "1/4", "1/8", "1/16", "1/32" };

/* ===== Demo Sequence State Machine ===== */
typedef enum {
    DEMO_IDLE = 0,
    DEMO_ZERO_MOTOR,
    DEMO_SET_MICROSTEP,
    DEMO_WAIT_MICROSTEP,
    DEMO_STRUM_1,
    DEMO_WAIT_3S_1,
    DEMO_MOVE_180,
    DEMO_WAIT_MOVE_DONE,
    DEMO_WAIT_3S_2,
    DEMO_HOME,
    DEMO_WAIT_HOME_DONE,
    DEMO_WAIT_SETTLE,
    DEMO_STRUM_2,
    DEMO_DONE
} DemoState_t;

static DemoState_t demo_state = DEMO_IDLE;
static uint32_t demo_timer = 0;

/* ===== Demo2: Smoke on the Water ===== */

/*
 * Base time unit (ms) - controls overall tempo.
 * All note durations and breaks are proportional to this value.
 * At 400ms base, one "1x" beat ≈ 150 BPM feel.
 * TODO: Tune this to match desired tempo for Smoke on the Water (~112 BPM).
 */
#define DEMO2_BASE_UNIT_MS   1000

#define DEMO2_NUM_NOTES      12

/* Solenoid parameters — TODO: calibrate duty & burst on hardware */
#define DEMO2_SOL_DUTY       50   /* Duty cycle % */
#define DEMO2_SOL_BURST      40   /* Burst cycles per strum */

/*
 * Each note in the riff uses 2 steppers + 2 solenoids (E and A strings).
 *
 * motor0_steps / motor1_steps: RELATIVE move from previous note position.
 *   Positive = forward (higher fret), negative = back toward nut.
 *   TODO: Calibrate these for your fret spacing + microstep mode.
 *
 * duration_x10: How long the note rings, as 10× multiplier of base unit.
 *   10 = 1.0×, 20 = 2.0×, 5 = 0.5×
 *
 * break_x10: Silence/travel time before the next note (same scale).
 */
typedef struct {
    int32_t motor0_steps;   /* Stepper 0 (E string) relative steps */
    int32_t motor1_steps;   /* Stepper 1 (A string) relative steps */
    uint8_t sol0_string;    /* Solenoid 0: string index (0=E2, 1=A2, ...) */
    uint8_t sol0_fret;      /* Solenoid 0: fret (0=open, 1-7) */
    uint8_t sol1_string;    /* Solenoid 1: string index */
    uint8_t sol1_fret;      /* Solenoid 1: fret (0=open, 1-7) */
    uint16_t duration_x10;  /* Note duration (10 = 1.0× base) */
    uint16_t break_x10;     /* Break after note (10 = 1.0× base) */
} Demo2Note_t;

/*
 * Smoke on the Water — main riff (4 phrases, 12 notes total)
 *
 *   Phrase 1:  n ─ n ─ n ──────── (long break)
 *   Phrase 2:  n ─ n ─ n · n ──── (short + long break)
 *   Phrase 3:  n ─ n ─ n ──────── (long break)
 *   Phrase 4:  n ─ n ──────────── (end)
 *
 * Proportional timing from user spec:
 *   Breaks:    1  1  3 | 1  1  0.5  3 | 1  1  3 | 1  0
 *   Durations: 1  1  2 | 1  1  0.5  2 | 1  1  2 | 1  2
 *
 * Motor steps are all 0 (placeholder) — fill in after calibrating
 * the number of steps needed to move between fret positions.
 *
 * Solenoid string/fret are all 0 (placeholder) — fill in after
 * determining the correct PWM frequency for each note.
 * sol0 = E string solenoid, sol1 = A string solenoid.
 */
static const Demo2Note_t demo2_notes[DEMO2_NUM_NOTES] = {
    /* ---- Phrase 1: 3 notes ---- */
    /* Note  0: First chord (open string position, no motor move) */
    { .motor0_steps = 0, .motor1_steps = 0, .sol0_string = 0, .sol0_fret = 0, .sol1_string = 1, .sol1_fret = 0, .duration_x10 = 10, .break_x10 = 10 },
    /* Note  1: Second chord (move steppers to next fret position) */
    { .motor0_steps = -1600, .motor1_steps = 0, .sol0_string = 0, .sol0_fret = 3, .sol1_string = 1, .sol1_fret = 0, .duration_x10 = 10, .break_x10 = 10 },
    /* Note  2: Third chord (held longer, followed by long break) */
    { .motor0_steps = -800, .motor1_steps = 0, .sol0_string = 0, .sol0_fret = 5, .sol1_string = 1, .sol1_fret = 0, .duration_x10 = 20, .break_x10 = 30 },

    /* ---- Phrase 2: 4 notes ---- */
    /* Note  3: Same as phrase 1 start (move back to first position) */
    { .motor0_steps = 2400, .motor1_steps = 0, .sol0_string = 0, .sol0_fret = 0, .sol1_string = 1, .sol1_fret = 0, .duration_x10 = 10, .break_x10 = 10 },
    /* Note  4: Second chord */
    { .motor0_steps = -1600, .motor1_steps = 0, .sol0_string = 0, .sol0_fret = 3, .sol1_string = 1, .sol1_fret = 0, .duration_x10 = 10, .break_x10 = 10 },
    /* Note  5: Quick grace note (half duration, half break) */
    { .motor0_steps = -1600, .motor1_steps = 0, .sol0_string = 0, .sol0_fret = 6, .sol1_string = 1, .sol1_fret = 0, .duration_x10 =  5, .break_x10 =  5 },
    /* Note  6: Resolve chord (held longer, followed by long break) */
    { .motor0_steps = 600, .motor1_steps = 0, .sol0_string = 0, .sol0_fret = 5, .sol1_string = 1, .sol1_fret = 0, .duration_x10 = 20, .break_x10 = 30 },

    /* ---- Phrase 3: 3 notes (same as phrase 1) ---- */
    /* Note  7: First chord */
    { .motor0_steps = 2600, .motor1_steps = 0, .sol0_string = 0, .sol0_fret = 0, .sol1_string = 1, .sol1_fret = 0, .duration_x10 = 10, .break_x10 = 10 },
    /* Note  8: Second chord */
    { .motor0_steps = -1600, .motor1_steps = 0, .sol0_string = 0, .sol0_fret = 3, .sol1_string = 1, .sol1_fret = 0, .duration_x10 = 10, .break_x10 = 10 },
    /* Note  9: Third chord (held longer, long break) */
    { .motor0_steps = -800, .motor1_steps = 0, .sol0_string = 0, .sol0_fret = 5, .sol1_string = 1, .sol1_fret = 0, .duration_x10 = 20, .break_x10 = 30 },

    /* ---- Phrase 4: 2 notes (ending) ---- */
    /* Note 10: Penultimate chord */
    { .motor0_steps = 800, .motor1_steps = 0, .sol0_string = 0, .sol0_fret = 3, .sol1_string = 1, .sol1_fret = 0, .duration_x10 = 10, .break_x10 = 10 },
    /* Note 11: Final chord (held long, no break — riff ends) */
    { .motor0_steps = 1600, .motor1_steps = 0, .sol0_string = 0, .sol0_fret = 0, .sol1_string = 1, .sol1_fret = 0, .duration_x10 = 20, .break_x10 =  0 },
};

typedef enum {
    DEMO2_IDLE = 0,
    DEMO2_INIT_ZERO_M0,      /* Zero motor 0 position */
    DEMO2_INIT_ZERO_M1,      /* Zero motor 1 position */
    DEMO2_INIT_MICROSTEP,    /* Set microstep for both motors */
    DEMO2_WAIT_INIT,         /* Brief settle after init */
    DEMO2_MOVE_MOTOR0,       /* Send relative move to stepper 0 */
    DEMO2_MOVE_MOTOR1,       /* Send relative move to stepper 1 */
    DEMO2_WAIT_MOTORS,       /* Wait for both steppers to finish */
    DEMO2_TRIGGER_SOL0,      /* Trigger solenoid 0 (E string) */
    DEMO2_WAIT_SOL0,         /* Brief wait for CM4 to process */
    DEMO2_TRIGGER_SOL1,      /* Trigger solenoid 1 (A string) */
    DEMO2_WAIT_DURATION,     /* Wait for note to ring out */
    DEMO2_WAIT_BREAK,        /* Silence between notes */
    DEMO2_DONE
} Demo2State_t;

static Demo2State_t demo2_state = DEMO2_IDLE;
static uint32_t     demo2_timer    = 0;
static uint8_t      demo2_note_idx = 0;

/* Forward declarations */
static void Cmd_Start(char *args);
static void Cmd_Speed(char *args);
static void Cmd_Accel(char *args);
static void Cmd_MaxSpeed(char *args);
static void Cmd_Move(char *args);
static void Cmd_Goto(char *args);
static void Cmd_Microstep(char *args);
static void Cmd_Enable(char *args);
static void Cmd_Disable(char *args);
static void Cmd_Zero(char *args);
static void Cmd_HomeCmd(char *args);
static void Cmd_Save(char *args);
static void Cmd_Delete(char *args);
static void Cmd_Presets(char *args);
static void Cmd_Stop(char *args);
static void Cmd_Status(char *args);
static void Cmd_Help(char *args);
static void Cmd_Demo(char *args);
static void Cmd_Demo2(char *args);
static void Demo_Tick(void);
static void Demo2_Tick(void);

/* Command table entry */
typedef struct {
    const char *name;
    void (*handler)(char *args);
    const char *help_text;
} CmdEntry_t;

static const CmdEntry_t command_table[] = {
    { "start",     Cmd_Start,     "start                   Start system (after homing)" },
    { "speed",     Cmd_Speed,     "speed <id> <steps/s>    Set motor speed (neg=reverse)" },
    { "accel",     Cmd_Accel,     "accel <id> <accel>      Set max acceleration" },
    { "maxspeed",  Cmd_MaxSpeed,  "maxspeed <id> <speed>   Set max speed limit" },
    { "move",      Cmd_Move,      "move <id> <steps>       Relative move (neg=reverse)" },
    { "goto",      Cmd_Goto,      "goto <id> <pos>|<name>  Absolute move or recall preset" },
    { "microstep", Cmd_Microstep, "microstep <id> <0-5>    Set microstep mode" },
    { "enable",    Cmd_Enable,    "enable <id>             Enable motor driver" },
    { "disable",   Cmd_Disable,   "disable <id>            Disable motor driver" },
    { "zero",      Cmd_Zero,      "zero <id>               Set current pos as new home (0)" },
    { "home",      Cmd_HomeCmd,   "home                    Move all motors back to pos 0" },
    { "save",      Cmd_Save,      "save <name> [id]        Save position as preset" },
    { "delete",    Cmd_Delete,    "delete <name>           Delete a saved preset" },
    { "presets",   Cmd_Presets,   "presets                  List saved presets" },
    { "stop",      Cmd_Stop,      "stop                    Emergency stop all" },
    { "status",    Cmd_Status,    "status                  Print motor status" },
    { "demo",      Cmd_Demo,      "demo                    Run solenoid+motor demo" },
    { "demo2",     Cmd_Demo2,     "demo2                   Smoke on the Water (2 strings)" },
    { "help",      Cmd_Help,      "help                    Show this help" },
    { NULL, NULL, NULL }
};

/* ===== Helpers ===== */

static int parse_motor_id(char *token, uint8_t *id_out)
{
    if (token == NULL) {
        printf("  Error: missing motor ID\r\n");
        return -1;
    }
    int val = atoi(token);
    if (val < 0 || val >= NUM_MOTORS) {
        printf("  Error: motor ID must be 0-%d\r\n", NUM_MOTORS - 1);
        return -1;
    }
    *id_out = (uint8_t)val;
    return 0;
}

static int parse_int32(char *token, int32_t *val_out, const char *param_name)
{
    if (token == NULL) {
        printf("  Error: missing %s\r\n", param_name);
        return -1;
    }
    *val_out = (int32_t)strtol(token, NULL, 10);
    return 0;
}

/**
 * @brief Check if system is started (for movement commands)
 * @return 1 if started, 0 if not (prints error message)
 */
static int check_started(void)
{
    if (!system_started) {
        printf("  Error: system not started. Type 'start' first.\r\n");
        return 0;
    }
    return 1;
}

/* ===== Public API ===== */

void SerialCmd_Init(void)
{
    system_started = 0;
    printf("[CMD] Motors homed. Type 'start' to begin, 'help' for commands.\r\n");
    printf("> ");
}

uint8_t SerialCmd_Process(void)
{
    /* Tick demo state machines every call (runs even when no command pending) */
    Demo_Tick();
    Demo2_Tick();

    if (!Debug_UART_IsLineReady()) {
        return 0;
    }

    Debug_UART_GetLine(cmd_buffer, CMD_BUFFER_SIZE);

    /* Extract command name */
    char *cmd_name = strtok(cmd_buffer, " \t");
    if (cmd_name == NULL) {
        printf("> ");
        return 1;
    }

    /* Search command table */
    for (const CmdEntry_t *entry = command_table; entry->name != NULL; entry++) {
        if (strcmp(cmd_name, entry->name) == 0) {
            entry->handler(NULL);  /* strtok state continues internally */
            printf("> ");
            return 1;
        }
    }

    printf("  Unknown command: '%s'. Type 'help' for list.\r\n", cmd_name);
    printf("> ");
    return 1;
}

/* ===== Command Handlers ===== */

static void Cmd_Start(char *args)
{
    (void)args;
    if (system_started) {
        printf("  System already started.\r\n");
        return;
    }
    system_started = 1;
    printf("  System STARTED - movement commands now active.\r\n");
}

static void Cmd_Speed(char *args)
{
    (void)args;
    uint8_t id;
    int32_t speed;
    char *tok = strtok(NULL, " \t");
    if (parse_motor_id(tok, &id) != 0) return;
    tok = strtok(NULL, " \t");
    if (parse_int32(tok, &speed, "speed") != 0) return;

    if (!check_started()) return;

    InterCore_SendMotorSpeed(id, speed);
    printf("  Motor %u speed set to %ld steps/s\r\n", id, (long)speed);
}

static void Cmd_Accel(char *args)
{
    (void)args;
    uint8_t id;
    int32_t accel;
    char *tok = strtok(NULL, " \t");
    if (parse_motor_id(tok, &id) != 0) return;
    tok = strtok(NULL, " \t");
    if (parse_int32(tok, &accel, "acceleration") != 0) return;

    InterCore_SendMotorAccel(id, accel);
    printf("  Motor %u max accel set to %ld\r\n", id, (long)accel);
}

static void Cmd_MaxSpeed(char *args)
{
    (void)args;
    uint8_t id;
    int32_t maxspd;
    char *tok = strtok(NULL, " \t");
    if (parse_motor_id(tok, &id) != 0) return;
    tok = strtok(NULL, " \t");
    if (parse_int32(tok, &maxspd, "max speed") != 0) return;

    InterCore_SendMotorMaxSpeed(id, maxspd);
    printf("  Motor %u max speed set to %ld\r\n", id, (long)maxspd);
}

static void Cmd_Move(char *args)
{
    (void)args;
    uint8_t id;
    int32_t steps;
    char *tok = strtok(NULL, " \t");
    if (parse_motor_id(tok, &id) != 0) return;
    tok = strtok(NULL, " \t");
    if (parse_int32(tok, &steps, "steps") != 0) return;

    if (!check_started()) return;

    InterCore_SendMotorMoveRelative(id, steps);
    printf("  Motor %u moving %ld steps\r\n", id, (long)steps);
}

static void Cmd_Goto(char *args)
{
    (void)args;
    char *tok = strtok(NULL, " \t");
    if (tok == NULL) {
        printf("  Usage: goto <id> <position>  OR  goto <preset_name>\r\n");
        return;
    }

    /* Check if first token is a number (motor ID) or a preset name */
    char *endptr;
    long first_val = strtol(tok, &endptr, 10);

    if (*endptr == '\0') {
        /* First token is a number — standard goto <id> <position> */
        uint8_t id = (uint8_t)first_val;
        if (first_val < 0 || first_val >= NUM_MOTORS) {
            printf("  Error: motor ID must be 0-%d\r\n", NUM_MOTORS - 1);
            return;
        }

        int32_t pos;
        tok = strtok(NULL, " \t");
        if (tok == NULL) {
            /* Only one number given — might be a mistyped preset? */
            printf("  Error: missing position. Use 'goto <id> <pos>' or 'goto <name>'\r\n");
            return;
        }
        if (parse_int32(tok, &pos, "position") != 0) return;

        if (!check_started()) return;

        InterCore_SendMotorGoto(id, pos);
        printf("  Motor %u moving to position %ld\r\n", id, (long)pos);
    } else {
        /* First token is a preset name */
        if (!check_started()) return;

        for (int i = 0; i < MAX_PRESETS; i++) {
            if (presets[i].used && strcmp(tok, presets[i].name) == 0) {
                InterCore_SendMotorGoto(presets[i].motor_id, presets[i].position);
                printf("  Recalling preset '%s': Motor %u to position %ld\r\n",
                       tok, presets[i].motor_id, (long)presets[i].position);
                return;
            }
        }
        printf("  Error: preset '%s' not found. Type 'presets' to list.\r\n", tok);
    }
}

static void Cmd_Microstep(char *args)
{
    (void)args;
    uint8_t id;
    int32_t mode;
    char *tok = strtok(NULL, " \t");
    if (parse_motor_id(tok, &id) != 0) return;
    tok = strtok(NULL, " \t");
    if (parse_int32(tok, &mode, "mode") != 0) return;
    if (mode < 0 || mode > 5) {
        printf("  Error: mode must be 0-5 (full/half/quarter/8th/16th/32nd)\r\n");
        return;
    }

    InterCore_SendMotorMicrostep(id, (uint8_t)mode);
    const char *mode_names[] = { "full", "half", "quarter", "8th", "16th", "32nd" };
    printf("  Motor %u microstep set to %s\r\n", id, mode_names[mode]);
}

static void Cmd_Enable(char *args)
{
    (void)args;
    uint8_t id;
    char *tok = strtok(NULL, " \t");
    if (parse_motor_id(tok, &id) != 0) return;

    InterCore_SendMotorEnable(id, 1);
    printf("  Motor %u enabled\r\n", id);
}

static void Cmd_Disable(char *args)
{
    (void)args;
    uint8_t id;
    char *tok = strtok(NULL, " \t");
    if (parse_motor_id(tok, &id) != 0) return;

    InterCore_SendMotorEnable(id, 0);
    printf("  Motor %u disabled\r\n", id);
}

static void Cmd_Zero(char *args)
{
    (void)args;
    uint8_t id;
    char *tok = strtok(NULL, " \t");
    if (parse_motor_id(tok, &id) != 0) return;

    InterCore_SendMotorZero(id);
    printf("  Motor %u position zeroed\r\n", id);
}

static void Cmd_HomeCmd(char *args)
{
    (void)args;
    if (!system_started) {
        printf("  Error: system not started. Type 'start' first.\r\n");
        return;
    }
    InterCore_HomeMotors();
    printf("  All motors returning to home (position 0).\r\n");
}

static void Cmd_Save(char *args)
{
    (void)args;
    char *tok = strtok(NULL, " \t");
    if (tok == NULL) {
        printf("  Usage: save <name> [motor_id]  (default motor_id=0)\r\n");
        return;
    }

    /* Copy preset name */
    char name[PRESET_NAME_LEN];
    strncpy(name, tok, PRESET_NAME_LEN - 1);
    name[PRESET_NAME_LEN - 1] = '\0';

    /* Optional motor ID (default 0) */
    uint8_t motor_id = 0;
    tok = strtok(NULL, " \t");
    if (tok != NULL) {
        if (parse_motor_id(tok, &motor_id) != 0) return;
    }

    /* Get current position from shared memory */
    int32_t pos = g_shared_memory.current_positions[motor_id];

    /* Check if preset name already exists — overwrite */
    for (int i = 0; i < MAX_PRESETS; i++) {
        if (presets[i].used && strcmp(name, presets[i].name) == 0) {
            presets[i].motor_id = motor_id;
            presets[i].position = pos;
            printf("  Preset '%s' updated: Motor %u at position %ld\r\n",
                   name, motor_id, (long)pos);
            return;
        }
    }

    /* Find empty slot */
    for (int i = 0; i < MAX_PRESETS; i++) {
        if (!presets[i].used) {
            strncpy(presets[i].name, name, PRESET_NAME_LEN - 1);
            presets[i].name[PRESET_NAME_LEN - 1] = '\0';
            presets[i].motor_id = motor_id;
            presets[i].position = pos;
            presets[i].used = 1;
            printf("  Preset '%s' saved: Motor %u at position %ld\r\n",
                   name, motor_id, (long)pos);
            return;
        }
    }

    printf("  Error: preset storage full (max %d). Use 'delete <name>' to free a slot.\r\n", MAX_PRESETS);
}

static void Cmd_Delete(char *args)
{
    (void)args;
    char *tok = strtok(NULL, " \t");
    if (tok == NULL) {
        printf("  Usage: delete <name>\r\n");
        return;
    }

    for (int i = 0; i < MAX_PRESETS; i++) {
        if (presets[i].used && strcmp(tok, presets[i].name) == 0) {
            presets[i].used = 0;
            printf("  Preset '%s' deleted.\r\n", tok);
            return;
        }
    }
    printf("  Error: preset '%s' not found.\r\n", tok);
}

static void Cmd_Presets(char *args)
{
    (void)args;
    printf("\r\n===== Saved Presets =====\r\n");
    uint8_t count = 0;
    for (int i = 0; i < MAX_PRESETS; i++) {
        if (presets[i].used) {
            uint8_t mid = presets[i].motor_id;
            int32_t pos = presets[i].position;
            uint8_t ustep = g_shared_memory.motor_microstep_status[mid];
            uint16_t mult = (ustep <= 5) ? microstep_multipliers[ustep] : 1;
            int32_t eff_spr = MOTOR_STEPS_PER_REV * mult;
            int32_t deg_x10 = (int32_t)((int64_t)pos * 3600 / eff_spr);
            int32_t deg_whole = deg_x10 / 10;
            int32_t deg_frac = deg_x10 % 10;
            if (deg_frac < 0) deg_frac = -deg_frac;

            printf("  %-12s  Motor %u  pos=%ld steps  (%ld.%ld deg)\r\n",
                   presets[i].name, mid, (long)pos, (long)deg_whole, (long)deg_frac);
            count++;
        }
    }
    if (count == 0) {
        printf("  (no presets saved)\r\n");
    }
    printf("========================\r\n");
}

static void Cmd_Stop(char *args)
{
    (void)args;
    /* Cancel demos if running */
    if (demo_state != DEMO_IDLE) {
        demo_state = DEMO_IDLE;
        printf("  Demo cancelled.\r\n");
    }
    if (demo2_state != DEMO2_IDLE) {
        demo2_state = DEMO2_IDLE;
        printf("  Demo2 cancelled.\r\n");
    }
    InterCore_EmergencyStop();
    printf("  EMERGENCY STOP - all motors stopped\r\n");
}

static void Cmd_Status(char *args)
{
    (void)args;
    printf("\r\n===== Motor Status =====\r\n");
    printf("System: %s\r\n", system_started ? "STARTED" : "STOPPED (type 'start')");
    printf("Status flags: 0x%08lX  Errors: 0x%08lX\r\n",
           (unsigned long)g_shared_memory.motor_status_flags,
           (unsigned long)g_shared_memory.motor_errors);
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        int32_t cur_spd = g_shared_memory.current_speeds[i];
        int32_t tgt_spd = g_shared_memory.target_speeds_status[i];
        int32_t pos = g_shared_memory.current_positions[i];
        int32_t max_spd = g_shared_memory.max_speeds_status[i];
        int32_t max_acc = g_shared_memory.max_accels_status[i];
        uint8_t enabled = g_shared_memory.motor_enabled[i];
        uint8_t ustep = g_shared_memory.motor_microstep_status[i];
        uint8_t moving = g_shared_memory.motor_is_moving[i];

        /* Skip unconfigured motors (max_speed == 0 means not configured) */
        if (max_spd == 0 && max_acc == 0 && !enabled) continue;

        const char *dir = (cur_spd > 0) ? "FWD" : (cur_spd < 0) ? "REV" : "---";
        const char *ustep_name = (ustep <= 5) ? microstep_names[ustep] : "?";

        /* Calculate degrees from home */
        uint16_t mult = (ustep <= 5) ? microstep_multipliers[ustep] : 1;
        int32_t effective_spr = MOTOR_STEPS_PER_REV * mult;
        int32_t deg_x10 = (int32_t)((int64_t)pos * 3600 / effective_spr);
        int32_t deg_whole = deg_x10 / 10;
        int32_t deg_frac = deg_x10 % 10;
        if (deg_frac < 0) deg_frac = -deg_frac;

        printf("  Motor %u: %s | %s | %s\r\n", i,
               enabled ? "ENABLED" : "DISABLED",
               moving ? "MOVING" : "STOPPED",
               dir);
        printf("    speed:  %ld / %ld (target: %ld) steps/s\r\n",
               (long)cur_spd, (long)max_spd, (long)tgt_spd);
        printf("    pos:    %ld steps  (%ld.%ld deg from home)\r\n",
               (long)pos, (long)deg_whole, (long)deg_frac);
        printf("    accel:  %ld steps/s^2\r\n", (long)max_acc);
        printf("    ustep:  %s\r\n", ustep_name);
    }
    printf("========================\r\n");
}

static void Cmd_Help(char *args)
{
    (void)args;
    printf("\r\n===== Motor Commands =====\r\n");
    for (const CmdEntry_t *entry = command_table; entry->name != NULL; entry++) {
        printf("  %s\r\n", entry->help_text);
    }
    printf("\r\nNote: movement commands (speed/move/goto) require 'start' first.\r\n");
    printf("==========================\r\n");
}

/* ===== Demo Command & State Machine ===== */

/**
 * @brief Trigger a solenoid via shared memory (same mechanism as debug_uart.c "sol" command)
 */
static void Demo_TriggerSolenoid(uint8_t sol_id, uint8_t string, uint8_t fret,
                                  uint8_t duty, uint16_t burst_cycles)
{
    g_shared_memory.solenoid_id = sol_id;
    g_shared_memory.solenoid_string = string;
    g_shared_memory.solenoid_fret = fret;
    g_shared_memory.solenoid_duty_cycle = duty;
    g_shared_memory.solenoid_activation_time_ms = 0;
    g_shared_memory.solenoid_burst_cycles = burst_cycles;
    g_shared_memory.solenoid_mode = SOLENOID_MODE_BURST;
    g_shared_memory.motor_command_flags |= CMD_TRIGGER_SOLENOID;
    InterCore_NotifyCM4();
}

static void Cmd_Demo(char *args)
{
    (void)args;

    if (demo_state != DEMO_IDLE) {
        printf("  Demo already running. Type 'stop' to cancel.\r\n");
        return;
    }

    /* Auto-start system if not started */
    if (!system_started) {
        system_started = 1;
        printf("  System auto-started for demo.\r\n");
    }

    printf("\r\n===== DEMO START =====\r\n");
    printf("  Sequence: strum E2 open -> wait 3s -> rotate 180deg -> wait 3s -> home -> strum E2 open\r\n\r\n");
    demo_state = DEMO_ZERO_MOTOR;
}

/**
 * @brief Demo state machine tick (called every main loop iteration ~1ms)
 *
 * Demo sequence:
 *   1. Zero motor 0 position counter
 *   2. Set microstep to 1/16 (mode 4)
 *   3. Strum E2 open string (solenoid 0, 50% duty, 40 burst cycles)
 *   4. Wait 3 seconds
 *   5. Move motor 0 by 1600 steps = 180 degrees at 1/16 microstepping
 *   6. Wait for move to complete
 *   7. Wait 3 seconds
 *   8. Home motor 0 (return to position 0)
 *   9. Wait for home to complete
 *  10. Strum E2 open string again (same as step 3)
 */
static void Demo_Tick(void)
{
    if (demo_state == DEMO_IDLE) {
        return;
    }

    uint32_t now = HAL_GetTick();

    switch (demo_state) {

    case DEMO_ZERO_MOTOR:
        InterCore_SendMotorZero(0);
        printf("  [Demo] Step 1: Motor 0 position zeroed\r\n");
        demo_state = DEMO_SET_MICROSTEP;
        break;

    case DEMO_SET_MICROSTEP:
        /* Mode 4 = 1/16 microstepping: 200 * 16 = 3200 steps/rev */
        InterCore_SendMotorMicrostep(0, 4);
        printf("  [Demo] Step 2: Microstep set to 1/16\r\n");
        demo_timer = now;
        demo_state = DEMO_WAIT_MICROSTEP;
        break;

    case DEMO_WAIT_MICROSTEP:
        /* Brief delay for microstep mode to apply on CM4 */
        if (now - demo_timer >= 50) {
            demo_state = DEMO_STRUM_1;
        }
        break;

    case DEMO_STRUM_1:
        /* Solenoid 0, E2 string (0), fret 0 (open), 50% duty, 40 burst cycles */
        Demo_TriggerSolenoid(0, 0, 0, 50, 40);
        printf("  [Demo] Step 3: Strumming E2 open (sol 0 0 0 50 0 40)\r\n");
        demo_timer = now;
        demo_state = DEMO_WAIT_3S_1;
        break;

    case DEMO_WAIT_3S_1:
        if (now - demo_timer >= 2000) {
            demo_state = DEMO_MOVE_180;
        }
        break;

    case DEMO_MOVE_180:
        /* 1600 steps at 1/16 microstepping = 180 degrees */
        InterCore_SendMotorMoveRelative(0, 1600);
        printf("  [Demo] Step 4: Moving 180 degrees (1600 steps at 1/16)\r\n");
        demo_state = DEMO_WAIT_MOVE_DONE;
        break;

    case DEMO_WAIT_MOVE_DONE:
        if (!g_shared_memory.motor_is_moving[0]) {
            printf("  [Demo] Move complete (pos: %ld steps)\r\n",
                   (long)g_shared_memory.current_positions[0]);
            demo_timer = now;
            demo_state = DEMO_WAIT_3S_2;
        }
        break;

    case DEMO_WAIT_3S_2:
        if (now - demo_timer >= 3000) {
            demo_state = DEMO_HOME;
        }
        break;

    case DEMO_HOME:
        InterCore_HomeMotors();
        printf("  [Demo] Step 5: Homing motor (returning to position 0)\r\n");
        demo_state = DEMO_WAIT_HOME_DONE;
        break;

    case DEMO_WAIT_HOME_DONE:
        if (!g_shared_memory.motor_is_moving[0]) {
            printf("  [Demo] Home reached (pos: %ld steps)\r\n",
                   (long)g_shared_memory.current_positions[0]);
            demo_timer = now;
            demo_state = DEMO_WAIT_SETTLE;
        }
        break;

    case DEMO_WAIT_SETTLE:
        /* Small settle time after homing before strumming */
        if (now - demo_timer >= 500) {
            demo_state = DEMO_STRUM_2;
        }
        break;

    case DEMO_STRUM_2:
        /* Same strum as before: E2 open, 50% duty, 40 cycles */
        Demo_TriggerSolenoid(0, 0, 0, 50, 40);
        printf("  [Demo] Step 6: Strumming E2 open again (sol 0 0 0 50 0 40)\r\n");
        demo_state = DEMO_DONE;
        break;

    case DEMO_DONE:
        printf("\r\n===== DEMO COMPLETE =====\r\n");
        printf("> ");
        demo_state = DEMO_IDLE;
        break;

    default:
        demo_state = DEMO_IDLE;
        break;
    }
}

/* ===== Demo2: Smoke on the Water — Command & State Machine ===== */

static void Cmd_Demo2(char *args)
{
    (void)args;

    if (demo2_state != DEMO2_IDLE) {
        printf("  Demo2 already running. Type 'stop' to cancel.\r\n");
        return;
    }

    /* Cancel demo1 if it happens to be running */
    if (demo_state != DEMO_IDLE) {
        demo_state = DEMO_IDLE;
        InterCore_EmergencyStop();
        printf("  Demo1 cancelled.\r\n");
    }

    /* Auto-start system if not started */
    if (!system_started) {
        system_started = 1;
        printf("  System auto-started for demo.\r\n");
    }

    printf("\r\n===== DEMO2: SMOKE ON THE WATER =====\r\n");
    printf("  2 steppers (motor 0 + 1) and 2 solenoids (E + A strings)\r\n");
    printf("  Base tempo unit: %u ms\r\n", DEMO2_BASE_UNIT_MS);
    printf("  12 notes across 4 phrases\r\n");
    printf("  NOTE: motor steps are placeholders — calibrate on hardware!\r\n\r\n");

    demo2_state = DEMO2_INIT_ZERO_M0;
}

/**
 * @brief Demo2 state machine tick — called every main loop iteration (~1ms)
 *
 * Plays the Smoke on the Water riff using a data-driven note table.
 * For each note the sequence is:
 *   1. Move stepper 0 (E string fret)  →  relative steps
 *   2. Move stepper 1 (A string fret)  →  relative steps
 *   3. Wait for both motors to finish
 *   4. Trigger solenoid 0 (E string pluck)
 *   5. Brief delay for CM4 to process
 *   6. Trigger solenoid 1 (A string pluck)
 *   7. Wait for note duration
 *   8. Wait for break (silence)
 *   9. Advance to next note or finish
 */
static void Demo2_Tick(void)
{
    if (demo2_state == DEMO2_IDLE) {
        return;
    }

    uint32_t now = HAL_GetTick();
    const Demo2Note_t *note = &demo2_notes[demo2_note_idx];

    switch (demo2_state) {

    /* ---------- Initialization ---------- */

    case DEMO2_INIT_ZERO_M0:
        InterCore_SendMotorZero(0);
        printf("  [Demo2] Init: Motor 0 position zeroed\r\n");
        demo2_state = DEMO2_INIT_ZERO_M1;
        break;

    case DEMO2_INIT_ZERO_M1:
        InterCore_SendMotorZero(1);
        printf("  [Demo2] Init: Motor 1 position zeroed\r\n");
        demo2_state = DEMO2_INIT_MICROSTEP;
        break;

    case DEMO2_INIT_MICROSTEP:
        /* 1/16 microstepping for both motors (mode 4 = 3200 steps/rev) */
        InterCore_SendMotorMicrostep(0, 4);
        InterCore_SendMotorMicrostep(1, 4);
        printf("  [Demo2] Init: Microstep 1/16 set for motors 0 & 1\r\n");
        demo2_timer = now;
        demo2_state = DEMO2_WAIT_INIT;
        break;

    case DEMO2_WAIT_INIT:
        if (now - demo2_timer >= 100) {
            demo2_note_idx = 0;
            printf("  [Demo2] Starting riff — %u notes to play\r\n\r\n",
                   DEMO2_NUM_NOTES);
            demo2_state = DEMO2_MOVE_MOTOR0;
        }
        break;

    /* ---------- Per-note: move steppers ---------- */

    case DEMO2_MOVE_MOTOR0:
        if (note->motor0_steps != 0) {
            InterCore_SendMotorMoveRelative(0, note->motor0_steps);
            printf("  [Demo2] Note %2u: Motor 0 → %ld steps\r\n",
                   demo2_note_idx, (long)note->motor0_steps);
        }
        demo2_timer = now;
        demo2_state = DEMO2_MOVE_MOTOR1;
        break;

    case DEMO2_MOVE_MOTOR1:
        /* Small delay between motor commands so CM4 processes the first */
        if (now - demo2_timer >= 10) {
            if (note->motor1_steps != 0) {
                InterCore_SendMotorMoveRelative(1, note->motor1_steps);
                printf("  [Demo2] Note %2u: Motor 1 → %ld steps\r\n",
                       demo2_note_idx, (long)note->motor1_steps);
            }
            demo2_state = DEMO2_WAIT_MOTORS;
        }
        break;

    case DEMO2_WAIT_MOTORS:
        /* Wait for both steppers to reach their target positions */
        if (!g_shared_memory.motor_is_moving[0] &&
            !g_shared_memory.motor_is_moving[1]) {
            demo2_state = DEMO2_TRIGGER_SOL0;
        }
        break;

    /* ---------- Per-note: trigger solenoids ---------- */

    case DEMO2_TRIGGER_SOL0:
        /* Solenoid 0 — string/fret read from the note table */
        Demo_TriggerSolenoid(0, note->sol0_string, note->sol0_fret,
                             DEMO2_SOL_DUTY, DEMO2_SOL_BURST);
        printf("  [Demo2] Note %2u: Sol 0 (str %u fret %u) triggered\r\n",
               demo2_note_idx, note->sol0_string, note->sol0_fret);
        demo2_timer = now;
        demo2_state = DEMO2_WAIT_SOL0;
        break;

    case DEMO2_WAIT_SOL0:
        /* Wait for CM4 to process solenoid 0 command before sending sol 1 */
        if (now - demo2_timer >= 10) {
            demo2_state = DEMO2_TRIGGER_SOL1;
        }
        break;

    case DEMO2_TRIGGER_SOL1:
        /* Solenoid 1 — string/fret read from the note table */
        Demo_TriggerSolenoid(1, note->sol1_string, note->sol1_fret,
                             DEMO2_SOL_DUTY, DEMO2_SOL_BURST);
        printf("  [Demo2] Note %2u: Sol 1 (str %u fret %u) triggered\r\n",
               demo2_note_idx, note->sol1_string, note->sol1_fret);
        demo2_timer = now;
        demo2_state = DEMO2_WAIT_DURATION;
        break;

    /* ---------- Per-note: timing ---------- */

    case DEMO2_WAIT_DURATION: {
        /* Let the note ring for its proportional duration */
        uint32_t dur_ms = (uint32_t)note->duration_x10 * DEMO2_BASE_UNIT_MS / 10;
        if (now - demo2_timer >= dur_ms) {
            if (note->break_x10 > 0) {
                demo2_timer = now;
                demo2_state = DEMO2_WAIT_BREAK;
            } else {
                /* No break after this note — advance or finish */
                demo2_note_idx++;
                if (demo2_note_idx >= DEMO2_NUM_NOTES) {
                    demo2_state = DEMO2_DONE;
                } else {
                    demo2_state = DEMO2_MOVE_MOTOR0;
                }
            }
        }
        break;
    }

    case DEMO2_WAIT_BREAK: {
        /* Silence between notes */
        uint32_t brk_ms = (uint32_t)note->break_x10 * DEMO2_BASE_UNIT_MS / 10;
        if (now - demo2_timer >= brk_ms) {
            demo2_note_idx++;
            if (demo2_note_idx >= DEMO2_NUM_NOTES) {
                demo2_state = DEMO2_DONE;
            } else {
                demo2_state = DEMO2_MOVE_MOTOR0;
            }
        }
        break;
    }

    /* ---------- Done ---------- */

    case DEMO2_DONE:
        printf("\r\n===== DEMO2 (Smoke on the Water) COMPLETE =====\r\n");
        printf("> ");
        demo2_state = DEMO2_IDLE;
        break;

    default:
        demo2_state = DEMO2_IDLE;
        break;
    }
}
