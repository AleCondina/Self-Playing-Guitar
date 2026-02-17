/**
 ******************************************************************************
 * @file    debug_uart.c
 * @brief   Debug UART utilities implementation
 ******************************************************************************
 */

#include "debug_uart.h"
#include "shared_memory.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

/* Use BSP COM port (initialized in main.c) */
extern UART_HandleTypeDef hcom_uart[COMn];

/* Command input buffer */
#define CMD_BUFFER_SIZE 64
static char cmd_buffer[CMD_BUFFER_SIZE];
static uint8_t cmd_index = 0;

/* Default values for motor tuning */
#define DEFAULT_MOTOR_SPEED     1000
#define DEFAULT_MOTOR_ACCEL     100
#define DEFAULT_STEPS_PER_MOVE  500
#define DEFAULT_SETTLE_TIME_MS  0

/**
 * @brief Initialize debug UART
 */
int32_t Debug_Init(void)
{
    /* UART is initialized in main.c via BSP_COM_Init() */
    /* This function is here for future expansion (e.g., DMA mode) */
    return 0;
}

/**
 * @brief Printf redirection to UART
 * @note  This is called by printf() - implements _write syscall
 */
int _write(int file, char *ptr, int len)
{
    (void)file; /* Unused parameter */

    /* Use BSP COM1 UART handle */
    if (HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t*)ptr, len, HAL_MAX_DELAY) == HAL_OK) {
        return len;
    } else {
        return -1;
    }
}

/**
 * @brief Print debug message
 */
void Debug_Printf(const char *fmt, ...)
{
    char buffer[256];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    printf("%s", buffer);
}

/**
 * @brief Print shared memory diagnostics
 */
void Debug_PrintSharedMemory(void)
{
    printf("\r\n===== Shared Memory Diagnostics =====\r\n");
    printf("CM7 Heartbeat: %lu\r\n", g_shared_memory.cm7_heartbeat);
    printf("CM4 Heartbeat: %lu\r\n", g_shared_memory.cm4_heartbeat);
    printf("CM4 Loop Hz:   %lu\r\n", g_shared_memory.cm4_loop_hz);
    printf("CM4 Overruns:  %lu\r\n", g_shared_memory.cm4_loop_overruns);
    printf("CM4 Max Time:  %lu us\r\n", g_shared_memory.cm4_max_loop_time_us);
    printf("Command Flags: 0x%08lX\r\n", g_shared_memory.motor_command_flags);
    printf("Status Flags:  0x%08lX\r\n", g_shared_memory.motor_status_flags);
    printf("Motor Errors:  0x%08lX\r\n", g_shared_memory.motor_errors);
    printf("CM7->CM4 Msgs: %lu\r\n", g_shared_memory.cm7_to_cm4_msg_count);
    printf("CM4->CM7 Msgs: %lu\r\n", g_shared_memory.cm4_to_cm7_msg_count);
    printf("=====================================\r\n\r\n");
}

/**
 * @brief Print motor status
 */
void Debug_PrintMotorStatus(uint8_t motor_id)
{
    if (motor_id >= NUM_MOTORS) {
        printf("Invalid motor ID: %u\r\n", motor_id);
        return;
    }

    printf("Motor %u: Speed=%ld steps/s, Pos=%ld steps\r\n",
           motor_id,
           g_shared_memory.current_speeds[motor_id],
           g_shared_memory.current_positions[motor_id]);
}

/**
 * @brief Print available commands
 */
void Debug_PrintHelp(void)
{
    printf("\r\n===== Motor Tuning Commands =====\r\n");
    printf("speed <value>    - Set max speed (steps/sec)\r\n");
    printf("accel <value>    - Set acceleration\r\n");
    printf("steps <value>    - Set steps per move\r\n");
    printf("settle <value>   - Set settling time (ms)\r\n");
    printf("microstep <0-5>  - Set microstep mode:\r\n");
    printf("                   0=full, 1=1/2, 2=1/4, 3=1/8, 4=1/16, 5=1/32\r\n");
    printf("start            - Start oscillation test\r\n");
    printf("stop             - Stop motor\r\n");
    printf("status           - Show current settings\r\n");
    printf("solhelp          - Solenoid commands help\r\n");
    printf("help             - Show this help\r\n");
    printf("=================================\r\n\r\n");
}

/**
 * @brief Print current motor tuning status
 */
void Debug_PrintTuningStatus(void)
{
    const char *microstep_names[] = {"Full", "1/2", "1/4", "1/8", "1/16", "1/32"};
    uint8_t ms_mode = g_shared_memory.motor_microstep_mode;
    if (ms_mode > 5) ms_mode = 0;

    printf("\r\n===== Motor Tuning Status =====\r\n");
    printf("Max Speed:    %ld steps/sec\r\n",
           g_shared_memory.motor_max_speed > 0 ? g_shared_memory.motor_max_speed : (int32_t)DEFAULT_MOTOR_SPEED);
    printf("Acceleration: %ld\r\n",
           g_shared_memory.motor_max_accel > 0 ? g_shared_memory.motor_max_accel : (int32_t)DEFAULT_MOTOR_ACCEL);
    printf("Steps/Move:   %ld\r\n",
           g_shared_memory.motor_steps_per_move > 0 ? g_shared_memory.motor_steps_per_move : (int32_t)DEFAULT_STEPS_PER_MOVE);
    printf("Settle Time:  %lu ms\r\n", g_shared_memory.motor_settle_time_ms);
    printf("Microstep:    %s (%u)\r\n", microstep_names[ms_mode], ms_mode);
    printf("Test Running: %s\r\n", g_shared_memory.motor_test_enabled ? "YES" : "NO");
    printf("Motor Moving: %s\r\n", (g_shared_memory.motor_status_flags & MOTOR_STATUS_MOVING) ? "YES" : "NO");
    printf("Position:     %ld steps\r\n", g_shared_memory.current_positions[0]);
    printf("Cur Speed:    %ld steps/sec\r\n", g_shared_memory.current_speeds[0]);
    printf("===============================\r\n\r\n");
}

/**
 * @brief Print solenoid help
 */
void Debug_PrintSolenoidHelp(void)
{
    printf("\r\n===== Solenoid Commands =====\r\n");
    printf("sol <id> <str> <fret> [duty] [time] [cycles]\r\n");
    printf("  id:     Solenoid 0-5\r\n");
    printf("  str:    String 0-5 (E2,A2,D3,G3,B3,E4)\r\n");
    printf("  fret:   Fret 0-7\r\n");
    printf("  duty:   Duty cycle 1-99%% (default: 50)\r\n");
    printf("  time:   Duration ms, 0=infinite (default: 0)\r\n");
    printf("  cycles: Burst N cycles, 0=disabled (default: 0)\r\n");
    printf("\r\n");
    printf("solstop <id|all>   - Stop solenoid(s)\r\n");
    printf("solstatus          - Show active solenoids\r\n");
    printf("solhelp            - Show this help\r\n");
    printf("==============================\r\n\r\n");
}

/**
 * @brief Print solenoid status
 */
void Debug_PrintSolenoidStatus(void)
{
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

/**
 * @brief Parse and execute a command
 * @param cmd Command string to parse
 */
static void Debug_ParseCommand(char *cmd)
{
    char *token;
    char *arg;
    int32_t value;

    /* Skip leading whitespace */
    while (*cmd == ' ' || *cmd == '\t') cmd++;

    /* Empty command */
    if (*cmd == '\0') return;

    /* Get command token */
    token = strtok(cmd, " \t");
    if (token == NULL) return;

    /* Get optional argument */
    arg = strtok(NULL, " \t");

    /* Parse commands */
    if (strcmp(token, "help") == 0 || strcmp(token, "?") == 0) {
        Debug_PrintHelp();
    }
    else if (strcmp(token, "status") == 0) {
        Debug_PrintTuningStatus();
    }
    else if (strcmp(token, "speed") == 0) {
        if (arg) {
            value = atoi(arg);
            if (value > 0 && value <= 10000) {
                g_shared_memory.motor_max_speed = value;
                printf("Speed set to %ld steps/sec\r\n", value);
            } else {
                printf("Invalid speed (1-10000)\r\n");
            }
        } else {
            printf("Usage: speed <value>\r\n");
        }
    }
    else if (strcmp(token, "accel") == 0) {
        if (arg) {
            value = atoi(arg);
            if (value > 0 && value <= 1000) {
                g_shared_memory.motor_max_accel = value;
                printf("Acceleration set to %ld\r\n", value);
            } else {
                printf("Invalid acceleration (1-1000)\r\n");
            }
        } else {
            printf("Usage: accel <value>\r\n");
        }
    }
    else if (strcmp(token, "steps") == 0) {
        if (arg) {
            value = atoi(arg);
            if (value > 0 && value <= 10000) {
                g_shared_memory.motor_steps_per_move = value;
                printf("Steps per move set to %ld\r\n", value);
            } else {
                printf("Invalid steps (1-10000)\r\n");
            }
        } else {
            printf("Usage: steps <value>\r\n");
        }
    }
    else if (strcmp(token, "settle") == 0) {
        if (arg) {
            value = atoi(arg);
            if (value >= 0 && value <= 5000) {
                g_shared_memory.motor_settle_time_ms = (uint32_t)value;
                printf("Settle time set to %ld ms\r\n", value);
            } else {
                printf("Invalid settle time (0-5000)\r\n");
            }
        } else {
            printf("Usage: settle <value>\r\n");
        }
    }
    else if (strcmp(token, "start") == 0) {
        g_shared_memory.motor_test_enabled = 1;
        printf("Motor oscillation test STARTED\r\n");
    }
    else if (strcmp(token, "stop") == 0) {
        g_shared_memory.motor_test_enabled = 0;
        printf("Motor oscillation test STOPPED\r\n");
    }
    else if (strcmp(token, "microstep") == 0) {
        if (arg) {
            value = atoi(arg);
            if (value >= 0 && value <= 5) {
                g_shared_memory.motor_microstep_mode = (uint8_t)value;
                g_shared_memory.motor_microstep_changed = 1;  /* Signal CM4 to apply */
                const char *names[] = {"Full step", "1/2 step", "1/4 step", "1/8 step", "1/16 step", "1/32 step"};
                printf("Microstep mode set to %s (%ld)\r\n", names[value], value);
            } else {
                printf("Invalid mode (0-5): 0=full, 1=1/2, 2=1/4, 3=1/8, 4=1/16, 5=1/32\r\n");
            }
        } else {
            printf("Usage: microstep <0-5>\r\n");
            printf("  0=full, 1=1/2, 2=1/4, 3=1/8, 4=1/16, 5=1/32\r\n");
        }
    }
    /* Solenoid commands */
    else if (strcmp(token, "solhelp") == 0) {
        Debug_PrintSolenoidHelp();
    }
    else if (strcmp(token, "solstatus") == 0) {
        Debug_PrintSolenoidStatus();
    }
    else if (strcmp(token, "sol") == 0) {
        /* Parse: sol <id> <string> <fret> [duty] [time_ms] [burst_cycles] */
        char *arg_id = arg;  /* Use already-parsed first argument */
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
    else if (strcmp(token, "solstop") == 0) {
        /* Use already-parsed first argument */
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
    else {
        printf("Unknown command: %s (type 'help' for commands)\r\n", token);
    }
}

/**
 * @brief Process UART input (non-blocking)
 * @note  Call this periodically from main loop
 */
void Debug_ProcessInput(void)
{
    uint8_t ch;

    /* Check for available data (non-blocking) */
    if (HAL_UART_Receive(&hcom_uart[COM1], &ch, 1, 0) == HAL_OK) {
        /* Echo character back */
        HAL_UART_Transmit(&hcom_uart[COM1], &ch, 1, 10);

        if (ch == '\r' || ch == '\n') {
            /* End of command - process it */
            printf("\r\n");
            cmd_buffer[cmd_index] = '\0';

            if (cmd_index > 0) {
                Debug_ParseCommand(cmd_buffer);
            }

            /* Reset buffer */
            cmd_index = 0;
            printf("> ");  /* Prompt */
        }
        else if (ch == '\b' || ch == 127) {
            /* Backspace */
            if (cmd_index > 0) {
                cmd_index--;
                printf(" \b");  /* Erase character on terminal */
            }
        }
        else if (cmd_index < CMD_BUFFER_SIZE - 1) {
            /* Add to buffer */
            cmd_buffer[cmd_index++] = ch;
        }
    }
}

/**
 * @brief Initialize motor tuning defaults in shared memory
 */
void Debug_InitMotorDefaults(void)
{
    /* Set default values if not already set */
    if (g_shared_memory.motor_max_speed == 0) {
        g_shared_memory.motor_max_speed = DEFAULT_MOTOR_SPEED;
    }
    if (g_shared_memory.motor_max_accel == 0) {
        g_shared_memory.motor_max_accel = DEFAULT_MOTOR_ACCEL;
    }
    if (g_shared_memory.motor_steps_per_move == 0) {
        g_shared_memory.motor_steps_per_move = DEFAULT_STEPS_PER_MOVE;
    }
    g_shared_memory.motor_settle_time_ms = DEFAULT_SETTLE_TIME_MS;
    g_shared_memory.motor_test_enabled = 1;  /* Start enabled by default */

    printf("\r\nMotor tuning initialized. Type 'help' for commands.\r\n");
    printf("> ");
}
