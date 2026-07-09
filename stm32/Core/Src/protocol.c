#include "protocol.h"
#include "mecanum.h"
#include "pid.h"
#include "motor.h"
#include "bode_collector.h"
#include "control_source.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* External arm command functions (defined in main.c) */
extern void cmd_execute_kms(const char *cmd);
extern void cmd_execute_dst(const char *cmd);
extern void cmd_execute_djr(const char *cmd);
extern void cmd_execute_drs(const char *cmd);
extern void cmd_execute_jnt(const char *cmd);
extern void cmd_execute_geta(const char *cmd);
extern void cmd_execute_set_servo(const char *cmd);

/* Deadzone test function */
void Deadzone_Test(void);

/* Command buffer */
static char cmd_buf[PROTOCOL_RX_BUF_SIZE];
static uint16_t cmd_buf_idx = 0;

/* TX buffer */
static char tx_buf[PROTOCOL_TX_BUF_SIZE];

/* Command velocity */
float Cmd_Vx = 0, Cmd_Vy = 0, Cmd_Vz = 0;

/* External UART handle */
extern UART_HandleTypeDef huart1;

/* Ring buffer variables (defined in stm32f4xx_it.c) */
#define RX_BUF_SIZE 256
extern uint8_t rx_buf[];
extern volatile uint16_t rx_head;
extern volatile uint16_t rx_tail;

/**
  * @brief  Initialize protocol
  * @param  None
  * @retval None
  */
void Protocol_Init(void)
{
    cmd_buf_idx = 0;
    memset(cmd_buf, 0, sizeof(cmd_buf));
    Cmd_Vx = 0;
    Cmd_Vy = 0;
    Cmd_Vz = 0;
    
    /* Enable USART1 RXNE interrupt for direct register access */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

/**
  * @brief  Send string via UART1
  * @param  str: String to send
  * @retval None
  */
void Protocol_SendString(const char *str)
{
    if(str == NULL) return;
    while(*str)
    {
        /* Direct register access for USART1 TX */
        USART1->DR = *str;
        while(!(USART1->SR & USART_SR_TXE));
        str++;
    }
}

/**
  * @brief  Parse float from string
  * @param  p: Pointer to string pointer (will be advanced)
  * @retval Parsed float value
  */
static float parse_float(const char **p)
{
    float val = 0.0f;
    float sign = 1.0f;
    float frac = 0.0f;
    float frac_div = 1.0f;
    
    if(**p == '-') { sign = -1.0f; (*p)++; }
    
    while(**p >= '0' && **p <= '9')
    {
        val = val * 10.0f + (float)(**p - '0');
        (*p)++;
    }
    
    if(**p == '.')
    {
        (*p)++;
        while(**p >= '0' && **p <= '9')
        {
            frac = frac * 10.0f + (float)(**p - '0');
            frac_div *= 10.0f;
            (*p)++;
        }
    }
    
    return sign * (val + frac / frac_div);
}

/**
  * @brief  Parse integer from string
  * @param  p: Pointer to string pointer (will be advanced)
  * @retval Parsed integer value
  */
static int parse_int(const char **p)
{
    int val = 0;
    int sign = 1;
    
    if(**p == '-') { sign = -1; (*p)++; }
    
    while(**p >= '0' && **p <= '9')
    {
        val = val * 10 + (int)(**p - '0');
        (*p)++;
    }
    
    return sign * val;
}

/**
  * @brief  Execute mecanum velocity command
  * @param  cmd: Command string
  * @retval None
  */
static void Cmd_Execute_MV(const char *cmd)
{
    const char *p = cmd;
    
    /* Find ':' after "$MV" */
    while(*p && *p != ':') p++;
    if(*p != ':')
    {
        Protocol_SendString("ERR: Usage: $MV:vx,vy,vz!\r\n");
        return;
    }
    p++;
    
    /* Parse vx, vy, vz */
    while(*p == ',') p++;
    Cmd_Vx = parse_float(&p);
    while(*p == ',') p++;
    Cmd_Vy = parse_float(&p);
    while(*p == ',') p++;
    Cmd_Vz = parse_float(&p);
    
    /* Switch control source to UART */
    Control_SetSource(CONTROL_SOURCE_UART);
    
    snprintf(tx_buf, sizeof(tx_buf), "OK: MV vx=%.2f vy=%.2f vz=%.2f\r\n", Cmd_Vx, Cmd_Vy, Cmd_Vz);
    Protocol_SendString(tx_buf);
}

/**
  * @brief  Execute mecanum stop command
  * @param  cmd: Command string
  * @retval None
  */
static void Cmd_Execute_MSTOP(const char *cmd)
{
    (void)cmd;
    
    /* Switch control source to UART */
    Control_SetSource(CONTROL_SOURCE_UART);
    
    Cmd_Vx = 0;
    Cmd_Vy = 0;
    Cmd_Vz = 0;
    
    /* Reset PID and stop motors immediately */
    PID_Reset();
    Set_Pwm(0, 0, 0, 0);
    
    Protocol_SendString("OK: Mecanum stopped\r\n");
}

/**
  * @brief  Execute PID parameter command
  * @param  cmd: Command string
  * @retval None
  * @note   Format: $MPID:kp,ki,kd! or $MPID:kp,ki!
  *         Ki自动补偿dt=10ms (Ki_discrete = Ki_continuous * 0.01)
  */
static void Cmd_Execute_MPID(const char *cmd)
{
    const char *p = cmd;
    float kp, ki, kd;
    
    /* Find ':' after "$MPID" */
    while(*p && *p != ':') p++;
    if(*p != ':')
    {
        Protocol_SendString("ERR: Usage: $MPID:kp,ki,kd!\r\n");
        Protocol_SendString("Note: Ki auto-compensates dt=10ms\r\n");
        return;
    }
    p++;
    
    /* Parse kp, ki, kd */
    kp = parse_float(&p);
    while(*p == ',') p++;
    ki = parse_float(&p);
    while(*p == ',') p++;
    kd = parse_float(&p);
    
    /* Auto-compensate dt for Ki */
    ki = ki * 0.01f;  /* Ki_discrete = Ki_continuous * dt */
    
    PID_SetParam(kp, ki);
    
    snprintf(tx_buf, sizeof(tx_buf), "OK: PID KP=%.1f KI=%.1f (raw=%.1f) KD=%.2f\r\n", 
             kp, ki, ki/0.01f, kd);
    Protocol_SendString(tx_buf);
}

/**
  * @brief  Execute encoder read command
  * @param  cmd: Command string
  * @retval None
  */
static void Cmd_Execute_MENC(const char *cmd)
{
    (void)cmd;

    /* Send latest encoder values from control loop.
       Do not read/reset counters here, or it will disturb PID sampling. */
    Protocol_SendEncoder();
}

/**
  * @brief  Execute status report command
  * @param  cmd: Command string
  * @retval None
  */
static void Cmd_Execute_MSTATUS(const char *cmd)
{
    (void)cmd;
    Protocol_SendStatus();
}

/**
  * @brief  Send encoder values
  * @param  None
  * @retval None
  */
void Protocol_SendEncoder(void)
{
    snprintf(tx_buf, sizeof(tx_buf), "#ENC:%.2f,%.2f,%.2f,%.2f!\r\n",
             Encoder_A, Encoder_B, Encoder_C, Encoder_D);
    Protocol_SendString(tx_buf);
}

/**
  * @brief  Send status report
  * @param  None
  * @retval None
  */
void Protocol_SendStatus(void)
{
    float kp, ki, kd;
    PID_GetParam(&kp, &ki, &kd);
    
    snprintf(tx_buf, sizeof(tx_buf),
             "#STATUS:Vx=%.2f,Vy=%.2f,Vz=%.2f,KP=%.1f,KI=%.1f,KD=%.2f!\r\n",
             Cmd_Vx, Cmd_Vy, Cmd_Vz, kp, ki, kd);
    Protocol_SendString(tx_buf);
}

/**
  * @brief  Trim leading whitespace from string
  */
static const char* trim_leading(const char *str)
{
    while(*str == ' ' || *str == '\t' || *str == '\r' || *str == '\n')
        str++;
    return str;
}

/**
  * @brief  Process received command
  * @param  cmd: Command string
  * @retval None
  */
static void Process_Command(const char *cmd)
{
    if(cmd == NULL || cmd[0] == '\0') return;
    
    /* Trim leading whitespace */
    cmd = trim_leading(cmd);
    
    if(cmd[0] == '$')
    {
        /* System command */
        char cmd_name[16];
        uint8_t i = 1, j = 0;
        
        while(cmd[i] != ':' && cmd[i] != '!' && cmd[i] != '\0' && j < 15)
        {
            cmd_name[j++] = cmd[i++];
        }
        cmd_name[j] = '\0';
        
        /* Mecanum commands */
        if(strcmp(cmd_name, "MV") == 0)
            Cmd_Execute_MV(cmd);
        else if(strcmp(cmd_name, "MSTOP") == 0)
            Cmd_Execute_MSTOP(cmd);
        else if(strcmp(cmd_name, "MPID") == 0)
            Cmd_Execute_MPID(cmd);
        else if(strcmp(cmd_name, "MENC") == 0)
            Cmd_Execute_MENC(cmd);
        else if(strcmp(cmd_name, "MSTATUS") == 0)
            Cmd_Execute_MSTATUS(cmd);
        /* Arm commands - local processing */
        else if(strcmp(cmd_name, "KMS") == 0)
            cmd_execute_kms(cmd);
        else if(strcmp(cmd_name, "DST") == 0)
            cmd_execute_dst(cmd);
        else if(strcmp(cmd_name, "DJR") == 0)
            cmd_execute_djr(cmd);
        else if(strcmp(cmd_name, "DRS") == 0)
            cmd_execute_drs(cmd);
        else if(strcmp(cmd_name, "JNT") == 0)
            cmd_execute_jnt(cmd);
        else if(strcmp(cmd_name, "GETA") == 0)
            cmd_execute_geta(cmd);
        /* Deadzone test command */
        else if(strcmp(cmd_name, "DEADZONE") == 0)
        {
            Deadzone_Test();
        }
        /* Motor test command - $TEST! forward, $TEST:stop! stop */
        else if(strcmp(cmd_name, "TEST") == 0)
        {
            const char *p = cmd;
            while(*p && *p != ':' && *p != '!') p++;
            if(*p == ':' && *(p+1) == 's') {
                /* $TEST:stop! - re-enable PID control */
                extern uint8_t test_mode;
                test_mode = 0;
                Set_Pwm(0, 0, 0, 0);
                Protocol_SendString("OK: Motors stopped, PID enabled\r\n");
            } else {
                /* $TEST! - bypass PID, direct PWM (A+, B+, C-, D-) */
                extern uint8_t test_mode;
                test_mode = 1;
                Set_Pwm(3000, 3000, -3000, -3000);
                Protocol_SendString("OK: All wheels forward (test mode)\r\n");
            }
        }
        /* Individual PWM control - $PWM:a,b,c,d! */
        else if(strcmp(cmd_name, "PWM") == 0)
        {
            const char *p = cmd;
            while(*p && *p != ':') p++;
            if(*p == ':') {
                p++;
                int pwm_a = parse_int(&p);
                while(*p == ',') p++;
                int pwm_b = parse_int(&p);
                while(*p == ',') p++;
                int pwm_c = parse_int(&p);
                while(*p == ',') p++;
                int pwm_d = parse_int(&p);
                
                /* Set test mode and apply PWM */
                extern uint8_t test_mode;
                test_mode = 1;
                Set_Pwm(pwm_a, pwm_b, pwm_c, pwm_d);
                
                snprintf(tx_buf, sizeof(tx_buf), "OK: PWM A=%d B=%d C=%d D=%d\r\n", 
                         pwm_a, pwm_b, pwm_c, pwm_d);
                Protocol_SendString(tx_buf);
            } else {
                Protocol_SendString("ERR: Usage: $PWM:a,b,c,d!\r\n");
            }
        }
        /* Bode plot commands (open loop PWM mode) */
        else if(strcmp(cmd_name, "BODE") == 0)
        {
            /* $BODE:freq,pwm_amp,pwm_bias,samples! */
            float freq, pwm_amp, pwm_bias;
            int samples;
            const char *p = cmd;
            while(*p && *p != ':') p++;
            if(*p == ':') {
                p++;
                freq = parse_float(&p);
                while(*p == ',') p++;
                pwm_amp = parse_float(&p);
                while(*p == ',') p++;
                pwm_bias = parse_float(&p);
                while(*p == ',') p++;
                samples = parse_int(&p);
                
                if(samples > BODE_MAX_SAMPLES) samples = BODE_MAX_SAMPLES;
                if(samples < 100) samples = 100;
                
                Bode_Start(freq, pwm_amp, pwm_bias, (uint16_t)samples);
                
                snprintf(tx_buf, sizeof(tx_buf), "OK: BODE freq=%.2f pwm_amp=%.0f pwm_bias=%.0f samples=%d\r\n", 
                         freq, pwm_amp, pwm_bias, samples);
                Protocol_SendString(tx_buf);
            }
        }
        else if(strcmp(cmd_name, "BSTOP") == 0)
        {
            Bode_Stop();
            Protocol_SendString("OK: Bode stopped\r\n");
        }
        else
        {
            snprintf(tx_buf, sizeof(tx_buf), "ERR: Unknown command '$%s'\r\n", cmd_name);
            Protocol_SendString(tx_buf);
        }
    }
    else if(cmd[0] == '#')
    {
        /* Servo control command - local processing */
        cmd_execute_set_servo(cmd);
    }
    else
    {
        Protocol_SendString("ERR: Invalid command format\r\n");
    }
}

/**
  * @brief  Process protocol (call in main loop)
  * @param  None
  * @retval None
  */
void Protocol_Process(void)
{
    /* Process all bytes in ring buffer */
    while(rx_tail != rx_head)
    {
        uint8_t ch = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
        
        /* Ignore CR/LF from serial terminal */
        if(ch == '\r' || ch == '\n')
            continue;
        
        /* Echo received character via direct register */
        USART1->DR = ch;
        while(!(USART1->SR & USART_SR_TXE));
        
        /* Accumulate command until '!' terminator */
        if(ch == '!')
        {
            cmd_buf[cmd_buf_idx] = '\0';
            
            /* Send response directly */
            const char *p = "\r\n[OK] CMD: ";
            while(*p) { USART1->DR = *p++; while(!(USART1->SR & USART_SR_TXE)); }
            
            p = cmd_buf;
            while(*p) { USART1->DR = *p++; while(!(USART1->SR & USART_SR_TXE)); }
            
            p = "\r\n";
            while(*p) { USART1->DR = *p++; while(!(USART1->SR & USART_SR_TXE)); }
            
            Process_Command(cmd_buf);
            cmd_buf_idx = 0;
            memset(cmd_buf, 0, sizeof(cmd_buf));
        }
        else if(cmd_buf_idx < PROTOCOL_RX_BUF_SIZE - 1)
        {
            cmd_buf[cmd_buf_idx++] = (char)ch;
        }
        else
        {
            /* Buffer overflow - reset */
            cmd_buf_idx = 0;
            memset(cmd_buf, 0, sizeof(cmd_buf));
        }
    }
}

/**
  * @brief  死区测定函数
  * @note   从0开始逐渐增加PWM，记录电机开始转动的阈值
  */
void Deadzone_Test(void)
{
    char msg[64];
    int pwm = 0;
    
    Protocol_SendString("\r\n=== Deadzone Test ===\r\n");
    Protocol_SendString("PWM -> Speed (m/s)\r\n");
    Protocol_SendString("-------------------\r\n");
    
    for(pwm = 0; pwm <= 3000; pwm += 100)
    {
        /* 设置PWM */
        Set_Pwm(pwm, pwm, pwm, pwm);
        HAL_Delay(300);  /* 等待电机响应 */
        
        /* 读取编码器 */
        extern void Get_Velocity_Form_Encoder(void);
        Get_Velocity_Form_Encoder();
        
        /* 发送结果 */
        extern float Encoder_A;
        snprintf(msg, sizeof(msg), "%4d -> %.3f\r\n", pwm, Encoder_A);
        Protocol_SendString(msg);
    }
    
    /* 停止电机 */
    Set_Pwm(0, 0, 0, 0);
    Protocol_SendString("=== Test Complete ===\r\n");
}
