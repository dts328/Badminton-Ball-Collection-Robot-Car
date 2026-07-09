/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "kinematics.h"
#include "servo.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "mecanum.h"
#include "imu.h"
#include "oled.h"
#include "led.h"
#include "battery.h"
#include "protocol.h"
#include "ros_protocol.h"
#include "bode_collector.h"
#include "control_source.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_BUF_SIZE    256
#define UART_TX_BUF_SIZE    512
#define SERVO_UPDATE_MS     20
#define DEAD_ZONE           250     /* PWM死区阈值 (实测: 200-300) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* UART receive buffer (interrupt-based, single char) */
volatile uint8_t  uart2_rx_char;
volatile uint8_t  uart2_rx_ready;

/* Command buffer (accumulates chars until '!' terminator) */
char     cmd_buf[UART_RX_BUF_SIZE];
uint16_t cmd_buf_idx;

/* Kinematics instance */
kinematics_t kinematics;

/* Servo update timing */
volatile uint32_t servo_tick_ms;

/* TX buffer for responses */
char tx_buf[UART_TX_BUF_SIZE];

/* Mecanum control timing */
extern volatile uint32_t tick_10ms;
static uint32_t tick_50ms_count = 0;  /* ROS send at 20Hz */
static uint32_t tick_500ms_count = 0;
static uint32_t tick_1s_count = 0;

/* Startup delay counter */
static uint32_t startup_delay = 0;

/* Test mode flag - bypass PID control */
uint8_t test_mode = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void uart_send_str(const char *str);
void uart_send_buf(const char *buf, uint16_t len);
void cmd_parse_and_execute(const char *cmd);
void cmd_execute_kms(const char *cmd);
void cmd_execute_dst(const char *cmd);
void cmd_execute_djr(const char *cmd);
void cmd_execute_drs(const char *cmd);
void cmd_execute_jnt(const char *cmd);
void cmd_execute_geta(const char *cmd);
void cmd_execute_set_servo(const char *cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();       /* Motor PWM - TIM1 */
  MX_TIM8_Init();       /* Servo PWM - TIM8 */
  MX_TIM9_Init();       /* Motor PWM - TIM9 */
  MX_TIM10_Init();      /* Motor PWM - TIM10 */
  MX_TIM11_Init();      /* Motor PWM - TIM11 */
  MX_TIM12_Init();      /* Servo PWM - TIM12 */
  MX_USART1_UART_Init(); /* Unified serial port */
  MX_USART2_UART_Init(); /* Debug serial (reserved) */
  MX_USART3_UART_Init(); /* ROS communication */
  /* USER CODE BEGIN 2 */

  /* Initialize encoders */
  Encoder_Init_All();
  
  /* Initialize motors */
  Motor_Init();
  
  /* Initialize PID */
  PID_Init();
  
  /* Initialize mecanum */
  Mecanum_Init();
  
  /* Initialize IMU (ICM20948) */
  IMU_Init();
  
  /* Initialize OLED */
  OLED_Init();
  
  /* Initialize LED and Buzzer */
  LED_Init();
  Buzzer_Init();
  
  /* Initialize battery voltage detection */
  Battery_Init();
  
  /* Initialize ROS protocol */
  ROS_Init();
  
  /* Initialize Bode collector */
  Bode_Init();
  
  /* Initialize protocol (UART1) */
  Protocol_Init();

  /* Note: USART2 interrupt receive disabled - using USART1 only */
  /* HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart2_rx_char, 1); */

  /* Setup kinematics with WeArm link lengths (mm) */
  setup_kinematics(150.0f, 105.0f, 97.0f, 155.0f, &kinematics);

  /* Calculate power-on position PWM: $KMS:0,200,80,2000! */
  kinematics_move(0, 200, 80, &kinematics);

  /* Initialize servo PWM with power-on position */
  servo_init(kinematics.servo_pwm);

  /* Reset command buffer */
  cmd_buf_idx = 0;
  memset(cmd_buf, 0, sizeof(cmd_buf));

  /* Initialize servo tick counter */
  servo_tick_ms = 0;

  /* Send startup banner via UART1 */
  Protocol_SendString("\r\n==================================================\r\n");
  Protocol_SendString("  WeArm + Mecanum Controller\r\n");
  Protocol_SendString("  STM32F407VET6 @ 168MHz\r\n");
  Protocol_SendString("==================================================\r\n");
  Protocol_SendString("  Arm Commands:\r\n");
  Protocol_SendString("    $KMS:x,y,z,t!   - IK move\r\n");
  Protocol_SendString("    #IDPpwmTtime!   - Servo control\r\n");
  Protocol_SendString("    $DST!           - Stop servos\r\n");
  Protocol_SendString("  Mecanum Commands:\r\n");
  Protocol_SendString("    $MV:vx,vy,vz!   - Set velocity (m/s)\r\n");
  Protocol_SendString("    $MSTOP!         - Stop motors\r\n");
  Protocol_SendString("    $MPID:kp,ki!    - Set PID\r\n");
  Protocol_SendString("    $MENC!          - Read encoders\r\n");
  Protocol_SendString("    $MSTATUS!       - Status report\r\n");
  Protocol_SendString("==================================================\r\n\r\n");
  
  /* Beep to indicate startup */
  Buzzer_Beep(100);
  
  /* Wait for ICM20948 stabilization (10 seconds) */
  Protocol_SendString("Waiting for IMU stabilization...\r\n");
  for(startup_delay = 0; startup_delay < 100; startup_delay++)
  {
      LED_Flash(10);
      HAL_Delay(100);
  }
  
  /* Set IMU zero offset */
  IMU_SetZero();
  Protocol_SendString("IMU ready.\r\n");
  
  Protocol_SendString("Arm ready.\r\n");
  
  /* Turn on LED to indicate ready */
  LED_HIGH();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ---- Protocol processing (UART1 - character commands) ---- */
    Protocol_Process();
    
    /* ---- ROS protocol processing (UART3 - binary frame) ---- */
    if(ros_rx_flag)
    {
        ROS_ProcessData();
        
        if(ROS_Control_Mode == ROS_MODE_MECANUM)
        {
            /* Use ROS velocity for mecanum control */
            Control_SetSource(CONTROL_SOURCE_ROS);
            Control_UpdateRosTime();
            
            Cmd_Vx = ROS_Move_X;
            Cmd_Vy = ROS_Move_Y;
            Cmd_Vz = ROS_Move_Z;
        }
        else if(ROS_Control_Mode == ROS_MODE_ARM)
        {
            /* ARM mode - update timestamp, stop base */
            Control_SetSource(CONTROL_SOURCE_ROS);
            Control_UpdateRosTime();
            
            Cmd_Vx = 0;
            Cmd_Vy = 0;
            Cmd_Vz = 0;
        }
    }
    
    /* ROS timeout protection (only in ROS control mode, one-shot) */
    {
        static uint8_t ros_timeout_active = 0;
        
        if(Control_GetSource() == CONTROL_SOURCE_ROS)
        {
            if(HAL_GetTick() - Control_GetRosLastTime() > 300)
            {
                if(!ros_timeout_active)
                {
                    ros_timeout_active = 1;
                    /* Timeout - stop motors, reset PID, switch back to UART */
                    Cmd_Vx = 0;
                    Cmd_Vy = 0;
                    Cmd_Vz = 0;
                    PID_Reset();
                    Set_Pwm(0, 0, 0, 0);
                    Control_SetSource(CONTROL_SOURCE_UART);
                }
            }
            else
            {
                ros_timeout_active = 0;
            }
        }
    }
    
    /* ---- Command processing (UART2 - reserved) ---- */
    if (uart2_rx_ready) {
        uint8_t ch = uart2_rx_char;
        uart2_rx_ready = 0;

        /* Re-enable interrupt receive IMMEDIATELY to avoid UART overrun */
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart2_rx_char, 1);

        /* Echo received character (blocking, but IRQ is already re-armed) */
        HAL_UART_Transmit(&huart2, &ch, 1, 10);

        /* Accumulate command until '!' terminator */
        if (ch == '!') {
            cmd_buf[cmd_buf_idx] = '\0';  /* null-terminate */
            uart_send_str("\r\n");
            cmd_parse_and_execute(cmd_buf);
            cmd_buf_idx = 0;
            memset(cmd_buf, 0, sizeof(cmd_buf));
        } else if (cmd_buf_idx < UART_RX_BUF_SIZE - 1) {
            cmd_buf[cmd_buf_idx++] = (char)ch;
        } else {
            /* Buffer overflow — reset */
            cmd_buf_idx = 0;
            memset(cmd_buf, 0, sizeof(cmd_buf));
        }
    }

    /* ---- Servo update (every 20ms) ---- */
    if (servo_tick_ms >= SERVO_UPDATE_MS) {
        servo_tick_ms = 0;
        servo_update();
    }

    /* ---- Mecanum control (every 5ms for 200Hz Bode) ---- */
    if (tick_10ms >= 5) {
        tick_10ms = 0;
        
        /* 0. Bode plot data collection */
        Bode_Process();
        
        /* 1. Read encoder speeds (always read, even in test mode) */
        Get_Velocity_Form_Encoder();
        
        /* Check if we should skip PID control */
        extern BodeConfig_t bode_config;
        extern uint8_t test_mode;
        uint8_t skip_pid = 0;
        
        if (bode_config.state == BODE_RUNNING) {
            skip_pid = 1;  /* Bode mode - open loop */
        }
        if (test_mode) {
            skip_pid = 1;  /* Test mode - direct PWM */
        }
        
        if (!skip_pid) {
            /* 2. Read IMU data */
            IMU_GetData();
            
            /* 3. Drive motor with inverse kinematics */
            Drive_Motor(Cmd_Vx, Cmd_Vy, Cmd_Vz);
            
            /* 4. PID calculation */
            Motor_A = Incremental_PI_A(Encoder_A, Target_A);
            Motor_B = Incremental_PI_B(Encoder_B, Target_B);
            Motor_C = Incremental_PI_C(Encoder_C, Target_C);
            Motor_D = Incremental_PI_D(Encoder_D, Target_D);
            
            /* 4.5 Deadzone compensation */
            if(Motor_A > 0 && Motor_A < DEAD_ZONE) Motor_A = DEAD_ZONE;
            if(Motor_A < 0 && Motor_A > -DEAD_ZONE) Motor_A = -DEAD_ZONE;
            if(Motor_B > 0 && Motor_B < DEAD_ZONE) Motor_B = DEAD_ZONE;
            if(Motor_B < 0 && Motor_B > -DEAD_ZONE) Motor_B = -DEAD_ZONE;
            if(Motor_C > 0 && Motor_C < DEAD_ZONE) Motor_C = DEAD_ZONE;
            if(Motor_C < 0 && Motor_C > -DEAD_ZONE) Motor_C = -DEAD_ZONE;
            if(Motor_D > 0 && Motor_D < DEAD_ZONE) Motor_D = DEAD_ZONE;
            if(Motor_D < 0 && Motor_D > -DEAD_ZONE) Motor_D = -DEAD_ZONE;
        
            /* 5. Limit PWM */
            if(Motor_A > 16700) Motor_A = 16700;
            if(Motor_A < -16700) Motor_A = -16700;
            if(Motor_B > 16700) Motor_B = 16700;
            if(Motor_B < -16700) Motor_B = -16700;
            if(Motor_C > 16700) Motor_C = 16700;
            if(Motor_C < -16700) Motor_C = -16700;
            if(Motor_D > 16700) Motor_D = 16700;
            if(Motor_D < -16700) Motor_D = -16700;
            
            /* 6. Stop when motor enable pin is inactive */
            if(!EN_IS_ACTIVE()) {
                /* Emergency stop */
                Motor_A = 0;
                Motor_B = 0;
                Motor_C = 0;
                Motor_D = 0;
            }
            
            /* 7. Set PWM output */
            /* Motor targets and encoder feedback already include per-wheel polarity. */
            Set_Pwm(Motor_A, Motor_B, -Motor_C, -Motor_D);
        } /* end if (!skip_pid) */
    }
    
    /* ---- ROS status send (every 50ms = 20Hz) ---- */
    tick_50ms_count++;
    if (tick_50ms_count >= 50) {
        tick_50ms_count = 0;
        
        /* Send ROS status frame */
        ROS_SendStatus();
    }
    
    /* ---- OLED update (every 500ms) ---- */
    tick_500ms_count++;
    if (tick_500ms_count >= 500) {
        tick_500ms_count = 0;
        
        /* Update OLED display */
        OLED_UpdateStatus();
    }
    
    /* ---- LED flash and battery check (every 1s) ---- */
    tick_1s_count++;
    if (tick_1s_count >= 1000) {
        tick_1s_count = 0;
        
        /* LED flash */
        LED_TOGGLE();
        
        /* Check battery voltage */
        if(Battery_IsLow()) {
            /* Low voltage warning */
            BUZZER_ON();
        } else {
            BUZZER_OFF();
        }
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* ======================================================================== */
/*  UART TX Helpers                                                         */
/* ======================================================================== */

void uart_send_str(const char *str)
{
    if (str == NULL) return;
    while(*str)
    {
        USART1->DR = *str;
        while(!(USART1->SR & USART_SR_TXE));
        str++;
    }
}

void uart_send_buf(const char *buf, uint16_t len)
{
    if (buf == NULL || len == 0) return;
    for(uint16_t i = 0; i < len; i++)
    {
        USART1->DR = buf[i];
        while(!(USART1->SR & USART_SR_TXE));
    }
}

/* ======================================================================== */
/*  Command Parser (dispatcher)                                             */
/* ======================================================================== */

void cmd_parse_and_execute(const char *cmd)
{
    if (cmd == NULL || cmd[0] == '\0') return;

    /*
     * Command format:
     *   $<CMD>!      — system commands (KMS, DST, DJR, DRS, JNT, GETA, etc.)
     *   #<ID>P...T...! — servo control commands
     */
    if (cmd[0] == '$') {
        /* System command */
        char cmd_name[8];
        uint8_t i = 1, j = 0;

        /* Extract command name (up to ':' or '!' or end) */
        while (cmd[i] != ':' && cmd[i] != '!' && cmd[i] != '\0' && j < 7) {
            cmd_name[j++] = cmd[i++];
        }
        cmd_name[j] = '\0';

        if (strcmp(cmd_name, "KMS") == 0) {
            cmd_execute_kms(cmd);
        } else if (strcmp(cmd_name, "DST") == 0) {
            cmd_execute_dst(cmd);
        } else if (strcmp(cmd_name, "DJR") == 0) {
            cmd_execute_djr(cmd);
        } else if (strcmp(cmd_name, "DRS") == 0) {
            cmd_execute_drs(cmd);
        } else if (strcmp(cmd_name, "JNT") == 0) {
            cmd_execute_jnt(cmd);
        } else if (strcmp(cmd_name, "GETA") == 0) {
            cmd_execute_geta(cmd);
        } else if (strcmp(cmd_name, "RST") == 0) {
            /* Software reset */
            uart_send_str("Resetting...\r\n");
            NVIC_SystemReset();
        } else {
            snprintf(tx_buf, sizeof(tx_buf),
                     "ERR: Unknown command '$%s'\r\n", cmd_name);
            uart_send_str(tx_buf);
        }
    } else if (cmd[0] == '#') {
        /* Servo control command */
        cmd_execute_set_servo(cmd);
    } else {
        uart_send_str("ERR: Invalid command format\r\n");
    }
}

/* ======================================================================== */
/*  Command: $KMS:x,y,z,time!  — Inverse Kinematics Move                    */
/* ======================================================================== */

/*
 * Manual float parser — avoids microlib sscanf("%f") limitation.
 * Parses a float (e.g. "353", "12.5", "-10") from *p, advances *p past it.
 */
static float parse_float(const char **p)
{
    float val = 0.0f;
    float sign = 1.0f;
    float frac = 0.0f;
    float frac_div = 1.0f;

    if (**p == '-') { sign = -1.0f; (*p)++; }
    /* Integer part */
    while (**p >= '0' && **p <= '9') {
        val = val * 10.0f + (float)(**p - '0');
        (*p)++;
    }
    /* Fractional part */
    if (**p == '.') {
        (*p)++;
        while (**p >= '0' && **p <= '9') {
            frac = frac * 10.0f + (float)(**p - '0');
            frac_div *= 10.0f;
            (*p)++;
        }
    }
    return sign * (val + frac / frac_div);
}

/* Manual int parser */
static int parse_int(const char **p)
{
    int val = 0;
    while (**p >= '0' && **p <= '9') {
        val = val * 10 + (int)(**p - '0');
        (*p)++;
    }
    return val;
}

void cmd_execute_kms(const char *cmd)
{
    float x, y, z;
    int time_ms = 1000;
    int result;
    const char *p;

    /* Find ':' after "$KMS" */
    p = cmd;
    while (*p && *p != ':') p++;
    if (*p != ':') {
        uart_send_str("ERR: Usage: $KMS:x,y,z,time!\r\n");
        return;
    }
    p++; /* skip ':' */

    /* Parse x, y, z, time — skip empty fields (consecutive commas) */
    while (*p == ',') p++;
    x = parse_float(&p);
    while (*p == ',') p++;
    y = parse_float(&p);
    while (*p == ',') p++;
    z = parse_float(&p);
    while (*p == ',') p++;
    if (*p >= '0' && *p <= '9') time_ms = parse_int(&p);

  /* Debug: use integer format to verify parsed values (avoids microlib %f issues) */
    snprintf(tx_buf, sizeof(tx_buf),
             "\r\n=== IK Move to (%d, %d, %d) mm, time=%d ms ===\r\n",
             (int)(x + 0.5f), (int)(y + 0.5f), (int)(z + 0.5f), time_ms);
    uart_send_str(tx_buf);

    /* Run inverse kinematics */
    result = kinematics_move(x, y, z, &kinematics);

    if (result) {
        /*
         * Solution found.
         * kinematics.servo_angle[] and kinematics.servo_pwm[] are populated.
         */
        snprintf(tx_buf, sizeof(tx_buf),
                 "IK solution found (wrist Alpha sweep)\r\n");
        uart_send_str(tx_buf);

        /* Report joint angles (as integer to avoid microlib %f) */
        snprintf(tx_buf, sizeof(tx_buf),
                 "Joint angles (deg):\r\n"
                 "  Base     (J0): %+4d  | PWM: %5d\r\n"
                 "  Shoulder (J1): %+4d  | PWM: %5d\r\n"
                 "  Elbow    (J2): %+4d  | PWM: %5d\r\n"
                 "  Wrist    (J3): %+4d  | PWM: %5d\r\n",
                 (int)(kinematics.servo_angle[0] + 0.5f), kinematics.servo_pwm[0],
                 (int)(kinematics.servo_angle[1] + 0.5f), kinematics.servo_pwm[1],
                 (int)(kinematics.servo_angle[2] + 0.5f), kinematics.servo_pwm[2],
                 (int)(kinematics.servo_angle[3] + 0.5f), kinematics.servo_pwm[3]);
        uart_send_str(tx_buf);

        /* Move the arm servos */
        servo_move_arm(kinematics.servo_pwm, (uint16_t)time_ms);

        snprintf(tx_buf, sizeof(tx_buf),
                 "Moving servos... (target PWM set, %d ms)\r\n"
                 "=== IK Move Complete ===\r\n\r\n",
                 time_ms);
        uart_send_str(tx_buf);
    } else {
        uart_send_str("ERR: No valid IK solution found!\r\n");
        uart_send_str("     Check: position may be out of workspace.\r\n");
        uart_send_str("     Workspace: Y=0~353mm, Z=-60~443mm\r\n");
    }
}

/* ======================================================================== */
/*  Command: $DST!  — Stop all servos                                       */
/* ======================================================================== */

void cmd_execute_dst(const char *cmd)
{
    (void)cmd;
    servo_stop(255);  /* 255 = stop all */
    uart_send_str("OK: All servos stopped\r\n");
}

/* ======================================================================== */
/*  Command: $DJR!  — Reset all servos to center                            */
/* ======================================================================== */

void cmd_execute_djr(const char *cmd)
{
    (void)cmd;
    servo_reset_all(2000);  /* Reset over 2 seconds */
    uart_send_str("OK: Resetting all servos to center (2000 ms)\r\n");
}

/* ======================================================================== */
/*  Command: $DRS!  — Report joint states                                   */
/* ======================================================================== */

void cmd_execute_drs(const char *cmd)
{
    (void)cmd;
    int len;

    /* Report arm configuration */
    snprintf(tx_buf, sizeof(tx_buf),
             "\r\nWeArm Configuration:\r\n"
             "  Link L0 (Base height): %d mm\r\n"
             "  Link L1 (Upper arm):   %d mm\r\n"
             "  Link L2 (Forearm):     %d mm\r\n"
             "  Link L3 (Gripper):     %d mm\r\n"
             "  Max horizontal reach:  %d mm\r\n"
             "  Max vertical reach:    %d mm\r\n"
             "\r\n",
             (int)(kinematics.L0 / 10),
             (int)(kinematics.L1 / 10),
             (int)(kinematics.L2 / 10),
             (int)(kinematics.L3 / 10),
             (int)((kinematics.L1 + kinematics.L2 + kinematics.L3) / 10),
             (int)((kinematics.L0 + kinematics.L1 + kinematics.L2 + kinematics.L3) / 10));
    uart_send_str(tx_buf);

    /* Report joint states */
    len = servo_get_state_string(tx_buf, sizeof(tx_buf));
    uart_send_buf(tx_buf, (uint16_t)len);
}

/* ======================================================================== */
/*  Command: $JNT!  — Report coordinate system info                         */
/* ======================================================================== */

void cmd_execute_jnt(const char *cmd)
{
    (void)cmd;
    int len;

    /* Report coordinate system description */
    len = servo_get_coord_sys_string(tx_buf, sizeof(tx_buf));
    uart_send_buf(tx_buf, (uint16_t)len);

    /* Report current joint states */
    uart_send_str("\r\nCurrent Joint States:\r\n");
    uart_send_str("=====================\r\n");

    snprintf(tx_buf, sizeof(tx_buf),
             "Joint 0 (Base):     PWM=%5d  |  Moving=%s\r\n"
             "Joint 1 (Shoulder): PWM=%5d  |  Moving=%s\r\n"
             "Joint 2 (Elbow):    PWM=%5d  |  Moving=%s\r\n"
             "Joint 3 (Wrist):    PWM=%5d  |  Moving=%s\r\n"
             "Joint 4 (Gripper):  PWM=%5d  |  Moving=%s\r\n"
             "Joint 5 (Spare):    PWM=%5d  |  Moving=%s\r\n",
             servo_state[0].cur, servo_state[0].inc ? "YES" : "NO",
             servo_state[1].cur, servo_state[1].inc ? "YES" : "NO",
             servo_state[2].cur, servo_state[2].inc ? "YES" : "NO",
             servo_state[3].cur, servo_state[3].inc ? "YES" : "NO",
             servo_state[4].cur, servo_state[4].inc ? "YES" : "NO",
             servo_state[5].cur, servo_state[5].inc ? "YES" : "NO");
    uart_send_str(tx_buf);
}

/* ======================================================================== */
/*  Command: $GETA!  — Ping / alive check                                   */
/* ======================================================================== */

void cmd_execute_geta(const char *cmd)
{
    (void)cmd;
    uart_send_str("WeArm OK\r\n");
}

/* ======================================================================== */
/*  Command: #<ID>P<PWM>T<TIME>!  — Direct servo control                    */
/*  Examples:                                                                */
/*    #000P1500T1000!   -> Servo 0 to 1500us in 1000ms                      */
/*    #255P1500T2000!   -> All servos to 1500us in 2000ms                    */
/*    #000PDST!         -> Stop servo 0                                      */
/* ======================================================================== */

void cmd_execute_set_servo(const char *cmd)
{
    uint16_t servo_id = 0;
    uint16_t pwm = 0;
    uint16_t time_ms = 0;
    const char *p;

    if (cmd[0] != '#') return;

    /* Parse servo ID: #XXX... */
    p = cmd + 1;
    servo_id = 0;
    while (*p >= '0' && *p <= '9') {
        servo_id = servo_id * 10 + (uint16_t)(*p - '0');
        p++;
    }

    /* Check for DST (stop) command */
    if (strncmp(p, "PDST", 4) == 0) {
        if (servo_id >= 255) {
            servo_stop(255);
            uart_send_str("OK: All servos stopped\r\n");
        } else if (servo_id < SERVO_NUM) {
            servo_stop((uint8_t)servo_id);
            snprintf(tx_buf, sizeof(tx_buf),
                     "OK: Servo %d stopped\r\n", servo_id);
            uart_send_str(tx_buf);
        }
        return;
    }

    /* Parse PWM: PXXXX */
    if (*p == 'P') {
        p++;
        pwm = 0;
        while (*p >= '0' && *p <= '9') {
            pwm = pwm * 10 + (uint16_t)(*p - '0');
            p++;
        }
    }

    /* Parse Time: TXXXX */
    if (*p == 'T') {
        p++;
        time_ms = 0;
        while (*p >= '0' && *p <= '9') {
            time_ms = time_ms * 10 + (uint16_t)(*p - '0');
            p++;
        }
    }

    /* Clamp values */
    if (pwm < 500)  pwm = 500;
    if (pwm > 2500) pwm = 2500;
    if (time_ms > 9999) time_ms = 9999;

    /* Execute */
    if (servo_id == 255) {
        /* All servos */
        uint8_t i;
        for (i = 0; i < SERVO_NUM; i++) {
            servo_move_to(i, (int16_t)pwm, time_ms);
        }
        snprintf(tx_buf, sizeof(tx_buf),
                 "OK: All servos -> PWM=%d, time=%d ms\r\n", pwm, time_ms);
        uart_send_str(tx_buf);
    } else if (servo_id < SERVO_NUM) {
        servo_move_to((uint8_t)servo_id, (int16_t)pwm, time_ms);
        snprintf(tx_buf, sizeof(tx_buf),
                 "OK: Servo %d -> PWM=%d, time=%d ms\r\n",
                 servo_id, pwm, time_ms);
        uart_send_str(tx_buf);
    } else {
        snprintf(tx_buf, sizeof(tx_buf),
                 "ERR: Invalid servo ID %d (max %d)\r\n",
                 servo_id, SERVO_NUM - 1);
        uart_send_str(tx_buf);
    }
}

/*
 * UART RX Complete callback (called by HAL when one byte is received
 * via HAL_UART_Receive_IT).
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        /* USART1 now uses ring buffer in ISR directly */
    }
    else if (huart->Instance == USART2) {
        uart2_rx_ready = 1;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
