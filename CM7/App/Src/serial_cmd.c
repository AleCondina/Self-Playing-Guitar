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
    DEMO_STRUM_E2,
    DEMO_WAIT_AFTER_E2,
    DEMO_MOVE_TO_G,
    DEMO_WAIT_MOVE_DONE,
    DEMO_WAIT_SETTLE_1,
    DEMO_STRUM_G,
    DEMO_WAIT_AFTER_G,
    DEMO_HOME,
    DEMO_WAIT_HOME_DONE,
    DEMO_WAIT_SETTLE_2,
    DEMO_STRUM_HOME,
    DEMO_DONE
} DemoState_t;

static DemoState_t demo_state = DEMO_IDLE;
static uint32_t demo_timer = 0;

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
static void Cmd_Sol(char *args);
static void Cmd_SolStop(char *args);
static void Cmd_SolStatus(char *args);
static void Cmd_Demo(char *args);
static void Demo_Tick(void);

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
    { "help",      Cmd_Help,      "help                    Show this help" },
    { "sol",       Cmd_Sol,       "sol <id> <str> <fret> [duty] [time] [cycles]  Trigger solenoid" },
    { "solstop",   Cmd_SolStop,   "solstop <id|all>        Stop solenoid(s)" },
    { "solstatus", Cmd_SolStatus, "solstatus               Show active solenoids" },
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
    /* Tick demo state machine every call (runs even when no command pending) */
    Demo_Tick();

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
    /* Cancel demo if running */
    if (demo_state != DEMO_IDLE) {
        demo_state = DEMO_IDLE;
        printf("  Demo cancelled.\r\n");
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

/* ===== Solenoid Commands ===== */

static void Cmd_Sol(char *args)
{
    /* Parse: sol <id> <string> <fret> [duty] [time_ms] [burst_cycles] */
    char *arg_id = strtok(args, " \t");
    char *arg_str = strtok(NULL, " \t");
    char *arg_fret = strtok(NULL, " \t");
    char *arg_duty = strtok(NULL, " \t");
    char *arg_time = strtok(NULL, " \t");
    char *arg_burst = strtok(NULL, " \t");

    /* Validate required args */
    if (!arg_id || !arg_str || !arg_fret) {
        printf("Usage: sol <id> <str> <fret> [duty] [time] [cycles]\r\n");
        return;
    }

    /* Parse and validate all parameters */
    int32_t id = atoi(arg_id);
    int32_t str = atoi(arg_str);
    int32_t fret = atoi(arg_fret);
    int32_t duty = arg_duty ? atoi(arg_duty) : 50;
    int32_t time_ms = arg_time ? atoi(arg_time) : 0;
    int32_t burst = arg_burst ? atoi(arg_burst) : 0;

    /* Validate ranges */
    if (id < 0 || id > 5) {
        printf("Invalid solenoid ID (0-5)\r\n");
        return;
    }
    if (str < 0 || str > 5) {
        printf("Invalid string (0-5)\r\n");
        return;
    }
    if (fret < 0 || fret > 7) {
        printf("Invalid fret (0-7)\r\n");
        return;
    }
    if (duty < 1 || duty > 99) {
        printf("Invalid duty cycle (1-99%%)\r\n");
        return;
    }
    if (time_ms < 0 || time_ms > 5000) {
        printf("Invalid time (0-5000ms)\r\n");
        return;
    }
    if (burst < 0 || burst > 1000) {
        printf("Invalid burst cycles (0-1000)\r\n");
        return;
    }

    /* Populate shared memory */
    g_shared_memory.solenoid_id = (uint8_t)id;
    g_shared_memory.solenoid_string = (uint8_t)str;
    g_shared_memory.solenoid_fret = (uint8_t)fret;
    g_shared_memory.solenoid_duty_cycle = (uint8_t)duty;
    g_shared_memory.solenoid_activation_time_ms = (uint16_t)time_ms;
    g_shared_memory.solenoid_burst_cycles = (uint16_t)burst;

    /* Determine mode */
    if (burst > 0) {
        g_shared_memory.solenoid_mode = SOLENOID_MODE_BURST;  /* BURST mode */
    } else if (time_ms > 0) {
        g_shared_memory.solenoid_mode = SOLENOID_MODE_TIMED;  /* TIMED mode */
    } else {
        g_shared_memory.solenoid_mode = SOLENOID_MODE_INFINITE;  /* INFINITE mode */
    }

    /* Set command flag */
    g_shared_memory.motor_command_flags |= CMD_TRIGGER_SOLENOID;

    printf("Triggering Sol%ld: Str%ld Fret%ld Duty%ld%% ", id, str, fret, duty);
    if (burst > 0) {
        printf("Burst=%ld cycles\r\n", burst);
    } else if (time_ms > 0) {
        printf("Time=%ldms\r\n", time_ms);
    } else {
        printf("Infinite\r\n");
    }
}

static void Cmd_SolStop(char *args)
{
    char *arg = strtok(args, " \t");

    if (!arg) {
        printf("Usage: solstop <id|all>\r\n");
        return;
    }

    if (strcmp(arg, "all") == 0) {
        g_shared_memory.solenoid_id = 0xFF;  /* Special value for "all" */
        printf("Stopping all solenoids\r\n");
    } else {
        int32_t id = atoi(arg);
        if (id < 0 || id > 5) {
            printf("Invalid solenoid ID (0-5)\r\n");
            return;
        }
        g_shared_memory.solenoid_id = (uint8_t)id;
        printf("Stopping solenoid %ld\r\n", id);
    }

    /* Set stop command flag */
    g_shared_memory.motor_command_flags |= CMD_STOP_SOLENOID;
}

static void Cmd_SolStatus(char *args)
{
    (void)args;

    printf("\r\n===== Solenoid Status =====\r\n");
    printf("Active Mask: 0x%02X\r\n", g_shared_memory.solenoid_active_mask);

    for (uint8_t i = 0; i < 6; i++) {
        if (g_shared_memory.solenoid_active_mask & (1 << i)) {
            printf("Solenoid %u: ACTIVE\r\n", i);
        }
    }

    printf("Last Config: Sol=%u Str=%u Fret=%u Duty=%u%%\r\n",
           g_shared_memory.solenoid_id,
           g_shared_memory.solenoid_string,
           g_shared_memory.solenoid_fret,
           g_shared_memory.solenoid_duty_cycle);
    printf("============================\r\n\r\n");
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
    printf("  Sequence: strum E2 -> move to G -> strum G -> home -> strum E2\r\n\r\n");
    demo_state = DEMO_ZERO_MOTOR;
}

/**
 * @brief Demo state machine tick (called every main loop iteration ~1ms)
 *
 * Demo sequence:
 *   1. Zero motor 0 position counter
 *   2. Set microstep to 1/16 (mode 4)
 *   3. Strum E2 open (solenoid 0, string 0, fret 0, 50% duty, 40 burst)
 *   4. Wait 1s, then move to G string (-1600 steps = 180 deg at 1/16)
 *   5. Wait for move, settle, then strum G (solenoid 0, string 0, fret 3)
 *   6. Wait 1s, then home motor 0
 *   7. Wait for home, settle, then strum E2 open again
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
            demo_state = DEMO_STRUM_E2;
        }
        break;

    case DEMO_STRUM_E2:
        /* Solenoid 0, string 0, fret 0 (open E2), 50% duty, 40 burst cycles */
        Demo_TriggerSolenoid(0, 0, 0, 50, 40);
        printf("  [Demo] Step 3: Strumming E2 open (sol 0 0 0 50 0 40)\r\n");
        demo_timer = now;
        demo_state = DEMO_WAIT_AFTER_E2;
        break;

    case DEMO_WAIT_AFTER_E2:
        if (now - demo_timer >= 1000) {
            demo_state = DEMO_MOVE_TO_G;
        }
        break;

    case DEMO_MOVE_TO_G:
        /* 1600 steps at 1/16 microstepping = 180 degrees to G string */
        InterCore_SendMotorMoveRelative(0, -1600);
        printf("  [Demo] Step 4: Moving to G string (-1600 steps / 180 deg)\r\n");
        demo_state = DEMO_WAIT_MOVE_DONE;
        break;

    case DEMO_WAIT_MOVE_DONE:
        if (!g_shared_memory.motor_is_moving[0]) {
            printf("  [Demo] Move complete (pos: %ld steps)\r\n",
                   (long)g_shared_memory.current_positions[0]);
            demo_timer = now;
            demo_state = DEMO_WAIT_SETTLE_1;
        }
        break;

    case DEMO_WAIT_SETTLE_1:
        /* Settle time before strumming */
        if (now - demo_timer >= 500) {
            demo_state = DEMO_STRUM_G;
        }
        break;

    case DEMO_STRUM_G:
        /* Solenoid 0, string 0, fret 3 (G), 50% duty, 40 burst cycles */
        Demo_TriggerSolenoid(0, 0, 3, 50, 40);
        printf("  [Demo] Step 5: Strumming G string (sol 0 0 3 50 0 40)\r\n");
        demo_timer = now;
        demo_state = DEMO_WAIT_AFTER_G;
        break;

    case DEMO_WAIT_AFTER_G:
        if (now - demo_timer >= 1000) {
            demo_state = DEMO_HOME;
        }
        break;

    case DEMO_HOME:
        InterCore_HomeMotors();
        printf("  [Demo] Step 6: Homing motor (returning to position 0)\r\n");
        demo_state = DEMO_WAIT_HOME_DONE;
        break;

    case DEMO_WAIT_HOME_DONE:
        if (!g_shared_memory.motor_is_moving[0]) {
            printf("  [Demo] Home reached (pos: %ld steps)\r\n",
                   (long)g_shared_memory.current_positions[0]);
            demo_timer = now;
            demo_state = DEMO_WAIT_SETTLE_2;
        }
        break;

    case DEMO_WAIT_SETTLE_2:
        /* Settle time after homing before strumming */
        if (now - demo_timer >= 500) {
            demo_state = DEMO_STRUM_HOME;
        }
        break;

    case DEMO_STRUM_HOME:
        /* Strum at home: string 0, fret 0 (open), 50% duty, 40 cycles */
        Demo_TriggerSolenoid(0, 0, 0, 50, 40);
        printf("  [Demo] Step 7: Strumming E2 at home (sol 0 0 0 50 0 40)\r\n");
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
