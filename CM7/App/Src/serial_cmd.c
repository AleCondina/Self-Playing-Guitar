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
static void Cmd_Presets(char *args);
static void Cmd_Stop(char *args);
static void Cmd_Status(char *args);
static void Cmd_Help(char *args);

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
    { "zero",      Cmd_Zero,      "zero <id>               Zero position counter" },
    { "home",      Cmd_HomeCmd,   "home                    Re-home all motors" },
    { "save",      Cmd_Save,      "save <name> [id]        Save position as preset" },
    { "presets",   Cmd_Presets,   "presets                  List saved presets" },
    { "stop",      Cmd_Stop,      "stop                    Emergency stop all" },
    { "status",    Cmd_Status,    "status                  Print motor status" },
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
    InterCore_HomeMotors();
    system_started = 0;
    printf("  All motors homed (positions zeroed). Type 'start' to resume.\r\n");
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

    printf("  Error: preset storage full (max %d). Delete presets or reboot.\r\n", MAX_PRESETS);
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
