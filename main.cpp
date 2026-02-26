/*
 * Single-file KSC project with full original business logic.
 * Merged from historical Core/Inc and Core/Src into one translation unit.
 */

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ==== Core/Inc/app_config.h ==== */
#include <stdint.h>

/* Optional features from the requirement document. */
#define ENABLE_BLUETOOTH            1U
#define ENABLE_LCD                  1U
#define ENABLE_MOTOR_PWM            1U

/*
 * 1: use conflict-free LCD remap so core motors + Bluetooth + LCD can run together.
 * 0: use requirement sheet LCD pins (may conflict with motor IN2 / USART2).
 */
#define LCD_USE_CONFLICT_FREE_PINS  1U

/* Main scheduling. */
#define MAIN_LOOP_PERIOD_MS         20U
#define LCD_REFRESH_PERIOD_MS       200U

/* OPB704 mark detection (A0). Active-low because collector is pulled up. */
#define OPB704_ACTIVE_LOW           1U
#define MARK_ADC_THRESHOLD          1800U
#define MARK_DEBOUNCE_MS            80U
#define MARK_REARM_MS               120U

/* 2Y0A21 25 cm threshold mapped to ADC count (12-bit @ 3.3V). */
#define OBSTACLE_ADC_THRESHOLD_25CM 1750U

/* Counter + display behavior (single common-cathode 7-seg digit). */
#define COUNTER_MAX_VALUE           9U

/* Motion timing (tune during on-car calibration). */
#define PAUSE_BEFORE_REVERSE_MS     250U
#define REVERSE_LONG_MS             1200U
#define BACKOFF_SHORT_MS            350U
#define TURN_90_MS                  560U
#define TURN_180_MS                 1080U

/* PWM speed setpoints (0..100). */
#define MOTOR_SPEED_FORWARD_PERCENT 72U
#define MOTOR_SPEED_REVERSE_PERCENT 62U
#define MOTOR_SPEED_TURN_PERCENT    60U

/* Buzzer feedback timings. */
#define BEEP_MARK_MS                50U
#define BEEP_OBSTACLE_MS            120U
#define BEEP_DONE_ON_MS             180U
#define BEEP_DONE_OFF_MS            80U

/* Optional UART report interval (HC-05). */
#define BLUETOOTH_STATUS_PERIOD_MS  500U

/* ==== Core/Inc/pin_map.h ==== */
/*
 * Mapping follows the requirement sheet.
 * Dx names correspond to Nucleo-style Arduino headers.
 */

/* L298N motor driver pins. */
#define MOTOR_IN1_GPIO_Port         GPIOA   /* D8  */
#define MOTOR_IN1_Pin               GPIO_PIN_9
#define MOTOR_IN2_GPIO_Port         GPIOA   /* D7  */
#define MOTOR_IN2_Pin               GPIO_PIN_8
#define MOTOR_IN3_GPIO_Port         GPIOB   /* D5  */
#define MOTOR_IN3_Pin               GPIO_PIN_4
#define MOTOR_IN4_GPIO_Port         GPIOB   /* D4  */
#define MOTOR_IN4_Pin               GPIO_PIN_5
#define MOTOR_ENA_GPIO_Port         GPIOC   /* D9  */
#define MOTOR_ENA_Pin               GPIO_PIN_7
#define MOTOR_ENB_GPIO_Port         GPIOB   /* D3  */
#define MOTOR_ENB_Pin               GPIO_PIN_3

/* Sensor ADC pins. */
#define OPB704_ADC_GPIO_Port        GPIOA   /* A0 */
#define OPB704_ADC_Pin              GPIO_PIN_0
#define OPB704_ADC_CHANNEL          ADC_CHANNEL_0

#define OBST_FRONT_ADC_GPIO_Port    GPIOA   /* A1 */
#define OBST_FRONT_ADC_Pin          GPIO_PIN_1
#define OBST_FRONT_ADC_CHANNEL      ADC_CHANNEL_1

#define OBST_LEFT_ADC_GPIO_Port     GPIOA   /* A2 */
#define OBST_LEFT_ADC_Pin           GPIO_PIN_4
#define OBST_LEFT_ADC_CHANNEL       ADC_CHANNEL_4

#define OBST_RIGHT_ADC_GPIO_Port    GPIOB   /* A3 */
#define OBST_RIGHT_ADC_Pin          GPIO_PIN_0
#define OBST_RIGHT_ADC_CHANNEL      ADC_CHANNEL_8

/* LED indicators. */
#define LED_OBSTACLE_GPIO_Port      GPIOA
#define LED_OBSTACLE_Pin            GPIO_PIN_5

/*
 * Requirement lists red LED on PA0, but PA0 is also OPB704 ADC input.
 * Default keeps OPB704 on PA0 and moves red LED to PC13.
 * If your board really wires red LED to PA0, set this to 1 and rework A0.
 */
#define RED_LED_SHARED_WITH_OPB704  0U
#if RED_LED_SHARED_WITH_OPB704
#define LED_MARK_GPIO_Port          GPIOA
#define LED_MARK_Pin                GPIO_PIN_0
#else
#define LED_MARK_GPIO_Port          GPIOC
#define LED_MARK_Pin                GPIO_PIN_13
#endif

/* 7-segment display (common cathode), segments A..G. */
#define SEG_A_GPIO_Port             GPIOC
#define SEG_A_Pin                   GPIO_PIN_0
#define SEG_B_GPIO_Port             GPIOC
#define SEG_B_Pin                   GPIO_PIN_1
#define SEG_C_GPIO_Port             GPIOC
#define SEG_C_Pin                   GPIO_PIN_2
#define SEG_D_GPIO_Port             GPIOC
#define SEG_D_Pin                   GPIO_PIN_3
#define SEG_E_GPIO_Port             GPIOC
#define SEG_E_Pin                   GPIO_PIN_4
#define SEG_F_GPIO_Port             GPIOC
#define SEG_F_Pin                   GPIO_PIN_5
#define SEG_G_GPIO_Port             GPIOC
#define SEG_G_Pin                   GPIO_PIN_6

/* Buzzer. */
#define BUZZER_GPIO_Port            GPIOB
#define BUZZER_Pin                  GPIO_PIN_10

/* HC-05 on USART2 (cross TX/RX externally as needed). */
#define HC05_UART_TX_GPIO_Port      GPIOA
#define HC05_UART_TX_Pin            GPIO_PIN_2
#define HC05_UART_RX_GPIO_Port      GPIOA
#define HC05_UART_RX_Pin            GPIO_PIN_3

/*
 * LCD1602 4-bit mode.
 * Default uses a conflict-free profile controlled by LCD_USE_CONFLICT_FREE_PINS.
 */
#define LCD_RS_GPIO_Port            GPIOC
#define LCD_RS_Pin                  GPIO_PIN_10
#define LCD_E_GPIO_Port             GPIOC
#define LCD_E_Pin                   GPIO_PIN_12

#if LCD_USE_CONFLICT_FREE_PINS
#define LCD_D4_GPIO_Port            GPIOB
#define LCD_D4_Pin                  GPIO_PIN_6
#define LCD_D5_GPIO_Port            GPIOB
#define LCD_D5_Pin                  GPIO_PIN_7
#define LCD_D6_GPIO_Port            GPIOA
#define LCD_D6_Pin                  GPIO_PIN_10
#define LCD_D7_GPIO_Port            GPIOB
#define LCD_D7_Pin                  GPIO_PIN_8
#define LCD_UART2_PA23_SHARED       0U
#else
#define LCD_D4_GPIO_Port            GPIOA
#define LCD_D4_Pin                  GPIO_PIN_3
#define LCD_D5_GPIO_Port            GPIOA
#define LCD_D5_Pin                  GPIO_PIN_2
#define LCD_D6_GPIO_Port            GPIOA
#define LCD_D6_Pin                  GPIO_PIN_10
#define LCD_D7_GPIO_Port            GPIOA
#define LCD_D7_Pin                  GPIO_PIN_8
#define LCD_UART2_PA23_SHARED       1U
#endif

/* ==== Core/Inc/motor.h ==== */
void Motor_Init(void);
void Motor_Enable(void);
void Motor_Disable(void);
void Motor_SetSpeed(uint8_t left_percent, uint8_t right_percent);
void Motor_Forward(void);
void Motor_Backward(void);
void Motor_TurnLeftInPlace(void);
void Motor_TurnRightInPlace(void);
void Motor_Stop(void);

void Motor_SetPwmChannels(
    TIM_HandleTypeDef *left_htim,
    uint32_t left_channel,
    TIM_HandleTypeDef *right_htim,
    uint32_t right_channel);

/* ==== Core/Inc/sensors.h ==== */
typedef struct
{
    uint16_t opb704_adc;
    uint16_t front_adc;
    uint16_t left_adc;
    uint16_t right_adc;
    uint8_t mark_detected;
    uint8_t front_blocked;
    uint8_t left_blocked;
    uint8_t right_blocked;
} SensorSnapshot;

void Sensors_Init(ADC_HandleTypeDef *hadc);
void Sensors_Update(void);
const SensorSnapshot *Sensors_GetSnapshot(void);

uint8_t Sensors_ConsumeMarkEdge(void);

/* ==== Core/Inc/seven_seg.h ==== */
#include <stdint.h>

void SevenSeg_Init(void);
void SevenSeg_ShowDigit(uint8_t digit);
void SevenSeg_ShowNumber(int32_t number);
void SevenSeg_Blank(void);

/* ==== Core/Inc/buzzer.h ==== */
#include <stdint.h>

void Buzzer_Init(void);
void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_BeepBlocking(uint16_t duration_ms);
void Buzzer_BeepPattern(uint8_t count, uint16_t on_ms, uint16_t off_ms);

/* ==== Core/Inc/indicators.h ==== */
#include <stdint.h>

void Indicators_Init(void);
void Indicators_SetObstacleLed(uint8_t on);
void Indicators_SetMarkLed(uint8_t on);
void Indicators_BlinkBoth(uint8_t count, uint16_t on_ms, uint16_t off_ms);

/* ==== Core/Inc/navigation.h ==== */
#include <stdint.h>

typedef enum
{
    NAV_SCENE_1_CLEAR_FORWARD = 1,
    NAV_SCENE_2_FRONT_ONLY = 2,
    NAV_SCENE_3_FRONT_LEFT = 3,
    NAV_SCENE_4_FRONT_RIGHT = 4,
    NAV_SCENE_5_FRONT_LEFT_RIGHT = 5
} NavSceneId;

typedef enum
{
    NAV_MOTION_STOP = 0,
    NAV_MOTION_FORWARD = 1,
    NAV_MOTION_BACKWARD = 2,
    NAV_MOTION_TURN_LEFT = 3,
    NAV_MOTION_TURN_RIGHT = 4
} NavMotion;

void Navigation_Init(void);
void Navigation_Process(void);

uint8_t Navigation_GetCounter(void);
NavSceneId Navigation_GetCurrentScene(void);
NavMotion Navigation_GetMotion(void);

/* ==== Core/Inc/lcd1602.h ==== */
#include <stdint.h>

void Lcd1602_Init(void);
void Lcd1602_Clear(void);
void Lcd1602_SetCursor(uint8_t row, uint8_t col);
void Lcd1602_Print(const char *text);
void Lcd1602_PrintLine(uint8_t row, const char *text16);

/* ==== Core/Inc/bluetooth.h ==== */
void Bluetooth_Init(UART_HandleTypeDef *huart);
void Bluetooth_SendText(const char *text);
void Bluetooth_SendStatus(uint8_t counter, uint8_t scene_id, const SensorSnapshot *snapshot);

/* ==== Core/Src/motor.c ==== */
static TIM_HandleTypeDef *g_left_pwm_timer = NULL;
static TIM_HandleTypeDef *g_right_pwm_timer = NULL;
static uint32_t g_left_pwm_channel = 0U;
static uint32_t g_right_pwm_channel = 0U;
static uint8_t g_pwm_ready = 0U;
static uint8_t g_motor_enabled = 0U;
static uint8_t g_left_speed_percent = 100U;
static uint8_t g_right_speed_percent = 100U;

static void Motor_WriteBridge(GPIO_PinState in1, GPIO_PinState in2, GPIO_PinState in3, GPIO_PinState in4)
{
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, in1);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, in2);
    HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, in3);
    HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, in4);
}

static void Motor_ApplyDuty(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t percent)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
    __HAL_TIM_SET_COMPARE(htim, channel, (period * percent) / 100U);
}

static void Motor_ApplyEnableState(void)
{
#if ENABLE_MOTOR_PWM
    if ((g_pwm_ready == 0U) || (g_left_pwm_timer == NULL) || (g_right_pwm_timer == NULL))
    {
        return;
    }

    if (g_motor_enabled != 0U)
    {
        Motor_ApplyDuty(g_left_pwm_timer, g_left_pwm_channel, g_left_speed_percent);
        Motor_ApplyDuty(g_right_pwm_timer, g_right_pwm_channel, g_right_speed_percent);
    }
    else
    {
        Motor_ApplyDuty(g_left_pwm_timer, g_left_pwm_channel, 0U);
        Motor_ApplyDuty(g_right_pwm_timer, g_right_pwm_channel, 0U);
    }
#endif
}

void Motor_SetPwmChannels(
    TIM_HandleTypeDef *left_htim,
    uint32_t left_channel,
    TIM_HandleTypeDef *right_htim,
    uint32_t right_channel)
{
    g_left_pwm_timer = left_htim;
    g_left_pwm_channel = left_channel;
    g_right_pwm_timer = right_htim;
    g_right_pwm_channel = right_channel;

#if ENABLE_MOTOR_PWM
    if ((g_left_pwm_timer != NULL) && (g_right_pwm_timer != NULL))
    {
        (void)HAL_TIM_PWM_Start(g_left_pwm_timer, g_left_pwm_channel);
        (void)HAL_TIM_PWM_Start(g_right_pwm_timer, g_right_pwm_channel);
        g_pwm_ready = 1U;
        Motor_ApplyEnableState();
    }
#else
    (void)left_htim;
    (void)right_htim;
    (void)left_channel;
    (void)right_channel;
#endif
}

void Motor_Init(void)
{
    Motor_WriteBridge(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
    Motor_Enable();
    Motor_SetSpeed(100U, 100U);
}

void Motor_Enable(void)
{
#if ENABLE_MOTOR_PWM
    g_motor_enabled = 1U;
    Motor_ApplyEnableState();
#else
    HAL_GPIO_WritePin(MOTOR_ENA_GPIO_Port, MOTOR_ENA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_ENB_GPIO_Port, MOTOR_ENB_Pin, GPIO_PIN_SET);
#endif
}

void Motor_Disable(void)
{
#if ENABLE_MOTOR_PWM
    g_motor_enabled = 0U;
    Motor_ApplyEnableState();
#else
    HAL_GPIO_WritePin(MOTOR_ENA_GPIO_Port, MOTOR_ENA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_ENB_GPIO_Port, MOTOR_ENB_Pin, GPIO_PIN_RESET);
#endif
}

void Motor_SetSpeed(uint8_t left_percent, uint8_t right_percent)
{
#if ENABLE_MOTOR_PWM
    if (left_percent > 100U)
    {
        left_percent = 100U;
    }
    if (right_percent > 100U)
    {
        right_percent = 100U;
    }

    g_left_speed_percent = left_percent;
    g_right_speed_percent = right_percent;
    Motor_ApplyEnableState();
#else
    (void)left_percent;
    (void)right_percent;
#endif
}

void Motor_Forward(void)
{
    Motor_Enable();
    Motor_WriteBridge(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);
}

void Motor_Backward(void)
{
    Motor_Enable();
    Motor_WriteBridge(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET);
}

void Motor_TurnLeftInPlace(void)
{
    Motor_Enable();
    Motor_WriteBridge(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_RESET);
}

void Motor_TurnRightInPlace(void)
{
    Motor_Enable();
    Motor_WriteBridge(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);
}

void Motor_Stop(void)
{
    Motor_WriteBridge(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
    Motor_Disable();
}

/* ==== Core/Src/sensors.c ==== */
static ADC_HandleTypeDef *g_adc = NULL;
static SensorSnapshot g_snapshot;

static uint16_t g_opb_filter = 0U;
static uint16_t g_front_filter = 0U;
static uint16_t g_left_filter = 0U;
static uint16_t g_right_filter = 0U;

static uint8_t g_mark_stable = 0U;
static uint8_t g_mark_candidate = 0U;
static uint32_t g_mark_candidate_since = 0U;
static uint32_t g_mark_last_edge_ms = 0U;
static uint8_t g_mark_edge_latched = 0U;

static uint16_t FilterIir(uint16_t previous, uint16_t input)
{
    if (previous == 0U)
    {
        return input;
    }
    return (uint16_t)(((uint32_t)previous * 3U + (uint32_t)input) / 4U);
}

static uint16_t Sensors_ReadChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef config = {0};

    if (g_adc == NULL)
    {
        return 0U;
    }

    config.Channel = channel;
    config.Rank = 1U;
    config.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    config.Offset = 0U;
    if (HAL_ADC_ConfigChannel(g_adc, &config) != HAL_OK)
    {
        return 0U;
    }

    if (HAL_ADC_Start(g_adc) != HAL_OK)
    {
        return 0U;
    }
    if (HAL_ADC_PollForConversion(g_adc, 5U) != HAL_OK)
    {
        (void)HAL_ADC_Stop(g_adc);
        return 0U;
    }

    {
        uint16_t value = (uint16_t)HAL_ADC_GetValue(g_adc);
        (void)HAL_ADC_Stop(g_adc);
        return value;
    }
}

static uint8_t IsMarkRawDetected(uint16_t adc_value)
{
#if OPB704_ACTIVE_LOW
    return (adc_value < MARK_ADC_THRESHOLD) ? 1U : 0U;
#else
    return (adc_value > MARK_ADC_THRESHOLD) ? 1U : 0U;
#endif
}

void Sensors_Init(ADC_HandleTypeDef *hadc)
{
    g_adc = hadc;

    g_snapshot.opb704_adc = 0U;
    g_snapshot.front_adc = 0U;
    g_snapshot.left_adc = 0U;
    g_snapshot.right_adc = 0U;
    g_snapshot.mark_detected = 0U;
    g_snapshot.front_blocked = 0U;
    g_snapshot.left_blocked = 0U;
    g_snapshot.right_blocked = 0U;

    g_mark_stable = 0U;
    g_mark_candidate = 0U;
    g_mark_candidate_since = HAL_GetTick();
    g_mark_last_edge_ms = 0U;
    g_mark_edge_latched = 0U;
}

void Sensors_Update(void)
{
    uint16_t raw_opb;
    uint16_t raw_front;
    uint16_t raw_left;
    uint16_t raw_right;
    uint8_t mark_raw;
    uint32_t now;

    raw_opb = Sensors_ReadChannel(OPB704_ADC_CHANNEL);
    raw_front = Sensors_ReadChannel(OBST_FRONT_ADC_CHANNEL);
    raw_left = Sensors_ReadChannel(OBST_LEFT_ADC_CHANNEL);
    raw_right = Sensors_ReadChannel(OBST_RIGHT_ADC_CHANNEL);

    g_opb_filter = FilterIir(g_opb_filter, raw_opb);
    g_front_filter = FilterIir(g_front_filter, raw_front);
    g_left_filter = FilterIir(g_left_filter, raw_left);
    g_right_filter = FilterIir(g_right_filter, raw_right);

    g_snapshot.opb704_adc = g_opb_filter;
    g_snapshot.front_adc = g_front_filter;
    g_snapshot.left_adc = g_left_filter;
    g_snapshot.right_adc = g_right_filter;

    g_snapshot.front_blocked = (g_snapshot.front_adc >= OBSTACLE_ADC_THRESHOLD_25CM) ? 1U : 0U;
    g_snapshot.left_blocked = (g_snapshot.left_adc >= OBSTACLE_ADC_THRESHOLD_25CM) ? 1U : 0U;
    g_snapshot.right_blocked = (g_snapshot.right_adc >= OBSTACLE_ADC_THRESHOLD_25CM) ? 1U : 0U;

    mark_raw = IsMarkRawDetected(g_snapshot.opb704_adc);
    now = HAL_GetTick();

    if (mark_raw != g_mark_candidate)
    {
        g_mark_candidate = mark_raw;
        g_mark_candidate_since = now;
    }

    if ((now - g_mark_candidate_since >= MARK_DEBOUNCE_MS) && (g_mark_stable != g_mark_candidate))
    {
        g_mark_stable = g_mark_candidate;
        if ((g_mark_stable != 0U) && (now - g_mark_last_edge_ms >= MARK_REARM_MS))
        {
            g_mark_edge_latched = 1U;
            g_mark_last_edge_ms = now;
        }
    }

    g_snapshot.mark_detected = g_mark_stable;
}

const SensorSnapshot *Sensors_GetSnapshot(void)
{
    return &g_snapshot;
}

uint8_t Sensors_ConsumeMarkEdge(void)
{
    uint8_t latched = g_mark_edge_latched;
    g_mark_edge_latched = 0U;
    return latched;
}

/* ==== Core/Src/seven_seg.c ==== */
/* Bit order: A B C D E F G */
static const uint8_t kDigitMask[10] =
{
    0x3FU, /* 0 */
    0x06U, /* 1 */
    0x5BU, /* 2 */
    0x4FU, /* 3 */
    0x66U, /* 4 */
    0x6DU, /* 5 */
    0x7DU, /* 6 */
    0x07U, /* 7 */
    0x7FU, /* 8 */
    0x6FU  /* 9 */
};

static GPIO_TypeDef *const kSegPorts[7] =
{
    SEG_A_GPIO_Port,
    SEG_B_GPIO_Port,
    SEG_C_GPIO_Port,
    SEG_D_GPIO_Port,
    SEG_E_GPIO_Port,
    SEG_F_GPIO_Port,
    SEG_G_GPIO_Port
};

static const uint16_t kSegPins[7] =
{
    SEG_A_Pin,
    SEG_B_Pin,
    SEG_C_Pin,
    SEG_D_Pin,
    SEG_E_Pin,
    SEG_F_Pin,
    SEG_G_Pin
};

static void SevenSeg_WriteMask(uint8_t mask)
{
    uint8_t idx;

    for (idx = 0U; idx < 7U; ++idx)
    {
        GPIO_PinState state = ((mask & (1U << idx)) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(kSegPorts[idx], kSegPins[idx], state);
    }
}

void SevenSeg_Init(void)
{
    SevenSeg_Blank();
}

void SevenSeg_ShowDigit(uint8_t digit)
{
    if (digit > 9U)
    {
        SevenSeg_Blank();
        return;
    }
    SevenSeg_WriteMask(kDigitMask[digit]);
}

void SevenSeg_ShowNumber(int32_t number)
{
    uint8_t digit = 0U;

    if (number < 0)
    {
        SevenSeg_Blank();
        return;
    }

    digit = (uint8_t)(number % 10);
    SevenSeg_ShowDigit(digit);
}

void SevenSeg_Blank(void)
{
    SevenSeg_WriteMask(0U);
}

/* ==== Core/Src/buzzer.c ==== */
void Buzzer_Init(void)
{
    Buzzer_Off();
}

void Buzzer_On(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
}

void Buzzer_Off(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

void Buzzer_BeepBlocking(uint16_t duration_ms)
{
    Buzzer_On();
    HAL_Delay(duration_ms);
    Buzzer_Off();
}

void Buzzer_BeepPattern(uint8_t count, uint16_t on_ms, uint16_t off_ms)
{
    uint8_t i;
    for (i = 0U; i < count; ++i)
    {
        Buzzer_On();
        HAL_Delay(on_ms);
        Buzzer_Off();
        if ((i + 1U) < count)
        {
            HAL_Delay(off_ms);
        }
    }
}

/* ==== Core/Src/indicators.c ==== */
void Indicators_Init(void)
{
    Indicators_SetObstacleLed(0U);
    Indicators_SetMarkLed(0U);
}

void Indicators_SetObstacleLed(uint8_t on)
{
    HAL_GPIO_WritePin(LED_OBSTACLE_GPIO_Port, LED_OBSTACLE_Pin, (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Indicators_SetMarkLed(uint8_t on)
{
    HAL_GPIO_WritePin(LED_MARK_GPIO_Port, LED_MARK_Pin, (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Indicators_BlinkBoth(uint8_t count, uint16_t on_ms, uint16_t off_ms)
{
    uint8_t i;
    for (i = 0U; i < count; ++i)
    {
        Indicators_SetObstacleLed(1U);
        Indicators_SetMarkLed(1U);
        HAL_Delay(on_ms);
        Indicators_SetObstacleLed(0U);
        Indicators_SetMarkLed(0U);
        if ((i + 1U) < count)
        {
            HAL_Delay(off_ms);
        }
    }
}

/* ==== Core/Src/bluetooth.c ==== */
static UART_HandleTypeDef *g_uart = NULL;

#if ENABLE_BLUETOOTH && ENABLE_LCD && LCD_UART2_PA23_SHARED
static void Bluetooth_ConfigPinsForUart(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = HC05_UART_TX_Pin | HC05_UART_RX_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);
}
#endif

void Bluetooth_Init(UART_HandleTypeDef *huart)
{
    g_uart = huart;
}

void Bluetooth_SendText(const char *text)
{
#if ENABLE_BLUETOOTH
    if ((g_uart == NULL) || (text == NULL))
    {
        return;
    }

#if ENABLE_LCD && LCD_UART2_PA23_SHARED
    Bluetooth_ConfigPinsForUart();
#endif
    (void)HAL_UART_Transmit(g_uart, (uint8_t *)text, (uint16_t)strlen(text), 100U);
#else
    (void)text;
#endif
}

void Bluetooth_SendStatus(uint8_t counter, uint8_t scene_id, const SensorSnapshot *snapshot)
{
#if ENABLE_BLUETOOTH
    char msg[128];
    int len;

    if ((g_uart == NULL) || (snapshot == NULL))
    {
        return;
    }

#if ENABLE_LCD && LCD_UART2_PA23_SHARED
    Bluetooth_ConfigPinsForUart();
#endif

    len = snprintf(
        msg,
        sizeof(msg),
        "scene=%u,cnt=%u,opb=%u,f=%u,l=%u,r=%u\r\n",
        (unsigned int)scene_id,
        (unsigned int)counter,
        (unsigned int)snapshot->opb704_adc,
        (unsigned int)snapshot->front_adc,
        (unsigned int)snapshot->left_adc,
        (unsigned int)snapshot->right_adc);

    if (len > 0)
    {
        (void)HAL_UART_Transmit(g_uart, (uint8_t *)msg, (uint16_t)len, 100U);
    }
#else
    (void)counter;
    (void)scene_id;
    (void)snapshot;
#endif
}

/* ==== Core/Src/lcd1602.c ==== */
#if ENABLE_LCD
static uint8_t g_lcd_pins_ready = 0U;

static void Lcd1602_DelayUs(uint32_t us)
{
    uint32_t loops;

    if (us == 0U)
    {
        return;
    }

    loops = (SystemCoreClock / 5000000U) * us;
    while (loops-- > 0U)
    {
        __NOP();
    }
}

static void Lcd1602_ConfigPinsForWrite(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = LCD_RS_Pin | LCD_E_Pin;
    HAL_GPIO_Init(LCD_RS_GPIO_Port, &gpio);

    gpio.Pin = LCD_D4_Pin;
    HAL_GPIO_Init(LCD_D4_GPIO_Port, &gpio);
    gpio.Pin = LCD_D5_Pin;
    HAL_GPIO_Init(LCD_D5_GPIO_Port, &gpio);
    gpio.Pin = LCD_D6_Pin;
    HAL_GPIO_Init(LCD_D6_GPIO_Port, &gpio);
    gpio.Pin = LCD_D7_Pin;
    HAL_GPIO_Init(LCD_D7_GPIO_Port, &gpio);

    g_lcd_pins_ready = 1U;
}

static void Lcd1602_EnsurePinsForWrite(void)
{
#if ENABLE_BLUETOOTH && LCD_UART2_PA23_SHARED
    Lcd1602_ConfigPinsForWrite();
#else
    if (g_lcd_pins_ready == 0U)
    {
        Lcd1602_ConfigPinsForWrite();
    }
#endif
}

static void Lcd1602_WriteDataPins(uint8_t nibble)
{
    HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, ((nibble & 0x01U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, ((nibble & 0x02U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, ((nibble & 0x04U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, ((nibble & 0x08U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Lcd1602_PulseEnable(void)
{
    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
    Lcd1602_DelayUs(2U);
    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
    Lcd1602_DelayUs(50U);
}

static void Lcd1602_SendNibble(uint8_t nibble)
{
    Lcd1602_WriteDataPins((uint8_t)(nibble & 0x0FU));
    Lcd1602_PulseEnable();
}

static void Lcd1602_SendByte(uint8_t value, uint8_t is_data)
{
    Lcd1602_EnsurePinsForWrite();
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, (is_data != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    Lcd1602_SendNibble((uint8_t)(value >> 4U));
    Lcd1602_SendNibble((uint8_t)(value & 0x0FU));
    Lcd1602_DelayUs(50U);
}

static void Lcd1602_SendCommand(uint8_t command)
{
    Lcd1602_SendByte(command, 0U);
}

static void Lcd1602_SendData(uint8_t data)
{
    Lcd1602_SendByte(data, 1U);
}

void Lcd1602_Init(void)
{
    Lcd1602_ConfigPinsForWrite();
    HAL_Delay(40U);

    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);

    Lcd1602_SendNibble(0x03U);
    HAL_Delay(5U);
    Lcd1602_SendNibble(0x03U);
    HAL_Delay(1U);
    Lcd1602_SendNibble(0x03U);
    HAL_Delay(1U);
    Lcd1602_SendNibble(0x02U);
    HAL_Delay(1U);

    Lcd1602_SendCommand(0x28U); /* 4-bit, 2 lines, 5x8 font */
    Lcd1602_SendCommand(0x0CU); /* display on, cursor off */
    Lcd1602_SendCommand(0x06U); /* entry mode */
    Lcd1602_Clear();
}

void Lcd1602_Clear(void)
{
    Lcd1602_SendCommand(0x01U);
    HAL_Delay(2U);
}

void Lcd1602_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t address = (row == 0U) ? 0x00U : 0x40U;
    if (col > 15U)
    {
        col = 15U;
    }
    Lcd1602_SendCommand((uint8_t)(0x80U | (address + col)));
}

void Lcd1602_Print(const char *text)
{
    uint8_t i = 0U;

    if (text == NULL)
    {
        return;
    }

    while ((text[i] != '\0') && (i < 16U))
    {
        Lcd1602_SendData((uint8_t)text[i]);
        ++i;
    }
}

void Lcd1602_PrintLine(uint8_t row, const char *text16)
{
    char buf[17];
    size_t len;

    if (text16 == NULL)
    {
        return;
    }

    memset(buf, ' ', sizeof(buf));
    buf[16] = '\0';

    len = strlen(text16);
    if (len > 16U)
    {
        len = 16U;
    }
    memcpy(buf, text16, len);

    Lcd1602_SetCursor(row, 0U);
    Lcd1602_Print(buf);
}
#else
void Lcd1602_Init(void) {}
void Lcd1602_Clear(void) {}
void Lcd1602_SetCursor(uint8_t row, uint8_t col)
{
    (void)row;
    (void)col;
}
void Lcd1602_Print(const char *text)
{
    (void)text;
}
void Lcd1602_PrintLine(uint8_t row, const char *text16)
{
    (void)row;
    (void)text16;
}
#endif

/* ==== Core/Src/navigation.c ==== */
typedef enum
{
    COUNT_MODE_UP = 0,
    COUNT_MODE_DOWN = 1
} CountMode;

typedef enum
{
    ACTION_NONE = 0,
    ACTION_PAUSE,
    ACTION_REVERSE,
    ACTION_BACKOFF,
    ACTION_TURN_LEFT_90,
    ACTION_TURN_RIGHT_90,
    ACTION_U_TURN_180
} ActionType;

typedef struct
{
    ActionType type;
    uint32_t duration_ms;
} TimedAction;

#define ACTION_QUEUE_CAPACITY 6U

static TimedAction g_action_queue[ACTION_QUEUE_CAPACITY];
static uint8_t g_action_count = 0U;

static TimedAction g_active_action;
static uint8_t g_active_action_valid = 0U;
static uint32_t g_active_action_start_ms = 0U;

static uint8_t g_counter = 0U;
static CountMode g_count_mode = COUNT_MODE_UP;
static NavSceneId g_scene = NAV_SCENE_1_CLEAR_FORWARD;
static NavMotion g_motion = NAV_MOTION_STOP;

static uint8_t g_halted = 0U;
static uint8_t g_scene5_countdown_mode = 0U;
static uint8_t g_scene2_turn_toggle = 0U;
static uint8_t g_last_front_blocked = 0U;

static void ActionQueue_Clear(void)
{
    g_action_count = 0U;
}

static uint8_t ActionQueue_Push(ActionType type, uint32_t duration_ms)
{
    if (g_action_count >= ACTION_QUEUE_CAPACITY)
    {
        return 0U;
    }

    g_action_queue[g_action_count].type = type;
    g_action_queue[g_action_count].duration_ms = duration_ms;
    ++g_action_count;
    return 1U;
}

static uint8_t ActionQueue_Pop(TimedAction *action)
{
    uint8_t i;

    if ((action == NULL) || (g_action_count == 0U))
    {
        return 0U;
    }

    *action = g_action_queue[0];

    for (i = 1U; i < g_action_count; ++i)
    {
        g_action_queue[i - 1U] = g_action_queue[i];
    }

    --g_action_count;
    return 1U;
}

static void StopWithCompleteSignal(void)
{
    g_halted = 1U;
    g_active_action_valid = 0U;
    ActionQueue_Clear();
    g_motion = NAV_MOTION_STOP;

    Motor_Stop();
    SevenSeg_ShowNumber(g_counter);
    Buzzer_BeepPattern(2U, BEEP_DONE_ON_MS, BEEP_DONE_OFF_MS);
    Indicators_BlinkBoth(2U, 80U, 80U);
}

static void HandleMarkEvent(void)
{
    if (Sensors_ConsumeMarkEdge() == 0U)
    {
        return;
    }

    Buzzer_BeepBlocking(BEEP_MARK_MS);

    if (g_count_mode == COUNT_MODE_UP)
    {
        if (g_counter < COUNTER_MAX_VALUE)
        {
            ++g_counter;
        }
    }
    else
    {
        if (g_counter > 0U)
        {
            --g_counter;
        }
    }

    SevenSeg_ShowNumber(g_counter);

    if ((g_scene5_countdown_mode != 0U) && (g_counter == 0U))
    {
        StopWithCompleteSignal();
    }
}

static void HandleFrontObstacleEdge(const SensorSnapshot *snapshot)
{
    if ((snapshot->front_blocked != 0U) && (g_last_front_blocked == 0U))
    {
        Buzzer_BeepBlocking(BEEP_OBSTACLE_MS);
    }

    g_last_front_blocked = snapshot->front_blocked;
}

static void ApplyAction(ActionType type)
{
    switch (type)
    {
    case ACTION_PAUSE:
        g_motion = NAV_MOTION_STOP;
        Motor_SetSpeed(0U, 0U);
        Motor_Stop();
        break;

    case ACTION_REVERSE:
    case ACTION_BACKOFF:
        g_motion = NAV_MOTION_BACKWARD;
        Motor_SetSpeed(MOTOR_SPEED_REVERSE_PERCENT, MOTOR_SPEED_REVERSE_PERCENT);
        Motor_Backward();
        g_count_mode = COUNT_MODE_DOWN;
        break;

    case ACTION_TURN_LEFT_90:
        g_motion = NAV_MOTION_TURN_LEFT;
        Motor_SetSpeed(MOTOR_SPEED_TURN_PERCENT, MOTOR_SPEED_TURN_PERCENT);
        Motor_TurnLeftInPlace();
        break;

    case ACTION_TURN_RIGHT_90:
    case ACTION_U_TURN_180:
        g_motion = NAV_MOTION_TURN_RIGHT;
        Motor_SetSpeed(MOTOR_SPEED_TURN_PERCENT, MOTOR_SPEED_TURN_PERCENT);
        Motor_TurnRightInPlace();
        break;

    default:
        g_motion = NAV_MOTION_STOP;
        Motor_SetSpeed(0U, 0U);
        Motor_Stop();
        break;
    }

    if ((type != ACTION_REVERSE) && (type != ACTION_BACKOFF))
    {
        if (g_scene5_countdown_mode != 0U)
        {
            g_count_mode = COUNT_MODE_DOWN;
        }
        else
        {
            g_count_mode = COUNT_MODE_UP;
        }
    }
}

static void StartNextActionIfIdle(void)
{
    if ((g_halted != 0U) || (g_active_action_valid != 0U))
    {
        return;
    }

    if (ActionQueue_Pop(&g_active_action) == 0U)
    {
        return;
    }

    g_active_action_valid = 1U;
    g_active_action_start_ms = HAL_GetTick();
    ApplyAction(g_active_action.type);
}

static void ProcessActiveAction(void)
{
    uint32_t now;

    if (g_active_action_valid == 0U)
    {
        return;
    }

    now = HAL_GetTick();
    if ((now - g_active_action_start_ms) < g_active_action.duration_ms)
    {
        return;
    }

    g_active_action_valid = 0U;
    g_motion = NAV_MOTION_STOP;
    Motor_Stop();
}

static void PlanScene2(void)
{
    ActionQueue_Clear();
    g_scene5_countdown_mode = 0U;

    (void)ActionQueue_Push(ACTION_PAUSE, PAUSE_BEFORE_REVERSE_MS);
    (void)ActionQueue_Push(ACTION_REVERSE, REVERSE_LONG_MS);
    if (g_scene2_turn_toggle == 0U)
    {
        (void)ActionQueue_Push(ACTION_TURN_LEFT_90, TURN_90_MS);
    }
    else
    {
        (void)ActionQueue_Push(ACTION_TURN_RIGHT_90, TURN_90_MS);
    }

    g_scene2_turn_toggle ^= 1U;
}

static void PlanScene3(void)
{
    ActionQueue_Clear();
    g_scene5_countdown_mode = 0U;

    (void)ActionQueue_Push(ACTION_BACKOFF, BACKOFF_SHORT_MS);
    (void)ActionQueue_Push(ACTION_TURN_RIGHT_90, TURN_90_MS);
}

static void PlanScene4(void)
{
    ActionQueue_Clear();
    g_scene5_countdown_mode = 0U;

    (void)ActionQueue_Push(ACTION_BACKOFF, BACKOFF_SHORT_MS);
    (void)ActionQueue_Push(ACTION_TURN_LEFT_90, TURN_90_MS);
}

static void PlanScene5(void)
{
    ActionQueue_Clear();
    g_scene5_countdown_mode = 1U;
    g_count_mode = COUNT_MODE_DOWN;

    (void)ActionQueue_Push(ACTION_BACKOFF, BACKOFF_SHORT_MS);
    (void)ActionQueue_Push(ACTION_U_TURN_180, TURN_180_MS);
}

void Navigation_Init(void)
{
    g_action_count = 0U;
    g_active_action_valid = 0U;
    g_counter = 0U;
    g_count_mode = COUNT_MODE_UP;
    g_scene = NAV_SCENE_1_CLEAR_FORWARD;
    g_motion = NAV_MOTION_STOP;
    g_halted = 0U;
    g_scene5_countdown_mode = 0U;
    g_scene2_turn_toggle = 0U;
    g_last_front_blocked = 0U;

    SevenSeg_ShowNumber(0);
    Motor_Stop();
}

void Navigation_Process(void)
{
    const SensorSnapshot *snapshot;
    uint8_t any_obstacle;

    Sensors_Update();
    snapshot = Sensors_GetSnapshot();

    any_obstacle = (uint8_t)((snapshot->front_blocked != 0U) ||
                             (snapshot->left_blocked != 0U) ||
                             (snapshot->right_blocked != 0U));
    Indicators_SetObstacleLed(any_obstacle);
    Indicators_SetMarkLed(snapshot->mark_detected);

    HandleMarkEvent();
    HandleFrontObstacleEdge(snapshot);

    if (g_halted != 0U)
    {
        g_motion = NAV_MOTION_STOP;
        Motor_Stop();
        return;
    }

    ProcessActiveAction();
    StartNextActionIfIdle();

    if ((g_active_action_valid != 0U) || (g_action_count != 0U))
    {
        return;
    }

    /*
     * Priority rule:
     * If front is clear, always go forward regardless of left/right obstacles.
     */
    if (snapshot->front_blocked == 0U)
    {
        g_scene = NAV_SCENE_1_CLEAR_FORWARD;
        if (g_scene5_countdown_mode != 0U)
        {
            g_count_mode = COUNT_MODE_DOWN;
            if (g_counter == 0U)
            {
                StopWithCompleteSignal();
                return;
            }
        }
        else
        {
            g_count_mode = COUNT_MODE_UP;
        }

        g_motion = NAV_MOTION_FORWARD;
        Motor_SetSpeed(MOTOR_SPEED_FORWARD_PERCENT, MOTOR_SPEED_FORWARD_PERCENT);
        Motor_Forward();
        return;
    }

    if ((snapshot->left_blocked == 0U) && (snapshot->right_blocked == 0U))
    {
        g_scene = NAV_SCENE_2_FRONT_ONLY;
        PlanScene2();
    }
    else if ((snapshot->left_blocked != 0U) && (snapshot->right_blocked == 0U))
    {
        g_scene = NAV_SCENE_3_FRONT_LEFT;
        PlanScene3();
    }
    else if ((snapshot->left_blocked == 0U) && (snapshot->right_blocked != 0U))
    {
        g_scene = NAV_SCENE_4_FRONT_RIGHT;
        PlanScene4();
    }
    else
    {
        g_scene = NAV_SCENE_5_FRONT_LEFT_RIGHT;
        PlanScene5();
    }

    StartNextActionIfIdle();
}

uint8_t Navigation_GetCounter(void)
{
    return g_counter;
}

NavSceneId Navigation_GetCurrentScene(void)
{
    return g_scene;
}

NavMotion Navigation_GetMotion(void)
{
    return g_motion;
}

/* ==== Core/Src/stm32f4xx_it.c ==== */
extern "C" void NMI_Handler(void)
{
}

extern "C" void HardFault_Handler(void)
{
    while (1)
    {
    }
}

extern "C" void MemManage_Handler(void)
{
    while (1)
    {
    }
}

extern "C" void BusFault_Handler(void)
{
    while (1)
    {
    }
}

extern "C" void UsageFault_Handler(void)
{
    while (1)
    {
    }
}

extern "C" void SVC_Handler(void)
{
}

extern "C" void DebugMon_Handler(void)
{
}

extern "C" void PendSV_Handler(void)
{
}

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* ==== Core/Src/stm32f4xx_hal_msp.c ==== */
extern "C" void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}

/* ==== Core/Src/main.c ==== */
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;
#if ENABLE_MOTOR_PWM
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
#endif

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
#if ENABLE_MOTOR_PWM
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
#endif
static void Error_Handler(void);

#if ENABLE_LCD
static const char *MotionToText(NavMotion motion)
{
    switch (motion)
    {
    case NAV_MOTION_FORWARD:
        return "FWD";
    case NAV_MOTION_BACKWARD:
        return "BACK";
    case NAV_MOTION_TURN_LEFT:
        return "L-TURN";
    case NAV_MOTION_TURN_RIGHT:
        return "R-TURN";
    default:
        return "STOP";
    }
}

static void Lcd_ShowStatus(void)
{
    const SensorSnapshot *snapshot = Sensors_GetSnapshot();
    char line1[17];
    char line2[17];

    (void)snprintf(
        line1,
        sizeof(line1),
        "S%u %-12s",
        (unsigned int)Navigation_GetCurrentScene(),
        MotionToText(Navigation_GetMotion()));

    (void)snprintf(
        line2,
        sizeof(line2),
        "C:%u F%dL%dR%d",
        (unsigned int)Navigation_GetCounter(),
        snapshot->front_blocked,
        snapshot->left_blocked,
        snapshot->right_blocked);

    Lcd1602_PrintLine(0U, line1);
    Lcd1602_PrintLine(1U, line2);
}
#endif

int main(void)
{
#if ENABLE_BLUETOOTH
    uint32_t last_bluetooth_report_ms = 0U;
#endif
#if ENABLE_LCD
    uint32_t last_lcd_refresh_ms = 0U;
#endif

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USART2_UART_Init();
#if ENABLE_MOTOR_PWM
    MX_TIM2_Init();
    MX_TIM3_Init();
    Motor_SetPwmChannels(&htim3, TIM_CHANNEL_2, &htim2, TIM_CHANNEL_2);
#endif

    Motor_Init();
    Sensors_Init(&hadc1);
    SevenSeg_Init();
    Buzzer_Init();
    Indicators_Init();
    Bluetooth_Init(&huart2);
#if ENABLE_LCD
    Lcd1602_Init();
#endif
    Navigation_Init();

#if ENABLE_BLUETOOTH
    Bluetooth_SendText("boot:navcar ready\r\n");
#endif

    while (1)
    {
        Navigation_Process();

#if ENABLE_BLUETOOTH
        if ((HAL_GetTick() - last_bluetooth_report_ms) >= BLUETOOTH_STATUS_PERIOD_MS)
        {
            Bluetooth_SendStatus(
                Navigation_GetCounter(),
                (uint8_t)Navigation_GetCurrentScene(),
                Sensors_GetSnapshot());
            last_bluetooth_report_ms = HAL_GetTick();
        }
#endif

#if ENABLE_LCD
        if ((HAL_GetTick() - last_lcd_refresh_ms) >= LCD_REFRESH_PERIOD_MS)
        {
            Lcd_ShowStatus();
            last_lcd_refresh_ms = HAL_GetTick();
        }
#endif

        HAL_Delay(MAIN_LOOP_PERIOD_MS);
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = 16U;
    osc.PLL.PLLN = 336U;
    osc.PLL.PLLP = RCC_PLLP_DIV4;
    osc.PLL.PLLQ = 7U;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    {
        Error_Handler();
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_ADC1_Init(void)
{
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1U;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

#if ENABLE_MOTOR_PWM
static void MX_TIM2_Init(void)
{
    TIM_OC_InitTypeDef oc = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 83U;     /* 84 MHz / (83+1) = 1 MHz */
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999U;       /* 1 kHz PWM */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0U;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_TIM3_Init(void)
{
    TIM_OC_InitTypeDef oc = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 83U;     /* 84 MHz / (83+1) = 1 MHz */
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999U;       /* 1 kHz PWM */
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0U;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
}
#endif

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, MOTOR_IN1_Pin | MOTOR_IN2_Pin | LED_OBSTACLE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN3_Pin | MOTOR_IN4_Pin | BUZZER_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin | SEG_G_Pin, GPIO_PIN_RESET);
#if !ENABLE_MOTOR_PWM
    HAL_GPIO_WritePin(MOTOR_ENA_GPIO_Port, MOTOR_ENA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_ENB_GPIO_Port, MOTOR_ENB_Pin, GPIO_PIN_RESET);
#endif
#if !RED_LED_SHARED_WITH_OPB704
    HAL_GPIO_WritePin(LED_MARK_GPIO_Port, LED_MARK_Pin, GPIO_PIN_RESET);
#endif
#if ENABLE_LCD
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, GPIO_PIN_RESET);
#endif

    gpio.Pin = MOTOR_IN1_Pin | MOTOR_IN2_Pin | LED_OBSTACLE_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = MOTOR_IN3_Pin | MOTOR_IN4_Pin | BUZZER_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = SEG_A_Pin | SEG_B_Pin | SEG_C_Pin | SEG_D_Pin | SEG_E_Pin | SEG_F_Pin | SEG_G_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);

#if !ENABLE_MOTOR_PWM
    gpio.Pin = MOTOR_ENA_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR_ENA_GPIO_Port, &gpio);

    gpio.Pin = MOTOR_ENB_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR_ENB_GPIO_Port, &gpio);
#endif

#if !RED_LED_SHARED_WITH_OPB704
    gpio.Pin = LED_MARK_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_MARK_GPIO_Port, &gpio);
#endif

#if ENABLE_LCD
    gpio.Pin = LCD_RS_Pin | LCD_E_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LCD_RS_GPIO_Port, &gpio);

    gpio.Pin = LCD_D4_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LCD_D4_GPIO_Port, &gpio);

    gpio.Pin = LCD_D5_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LCD_D5_GPIO_Port, &gpio);

    gpio.Pin = LCD_D6_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LCD_D6_GPIO_Port, &gpio);

    gpio.Pin = LCD_D7_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LCD_D7_GPIO_Port, &gpio);
#endif

    gpio.Pin = OPB704_ADC_Pin | OBST_FRONT_ADC_Pin | OBST_LEFT_ADC_Pin;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = OBST_RIGHT_ADC_Pin;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);

#if ENABLE_BLUETOOTH
#if !(ENABLE_LCD && LCD_UART2_PA23_SHARED)
    gpio.Pin = HC05_UART_TX_Pin | HC05_UART_RX_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);
#endif
#endif
}

extern "C" void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{
    if (adcHandle->Instance == ADC1)
    {
        __HAL_RCC_ADC1_CLK_ENABLE();
    }
}

extern "C" void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{
    if (uartHandle->Instance == USART2)
    {
        __HAL_RCC_USART2_CLK_ENABLE();
    }
}

#if ENABLE_MOTOR_PWM
extern "C" void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *tim_pwmHandle)
{
    GPIO_InitTypeDef gpio = {0};

    if (tim_pwmHandle->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();

        gpio.Pin = MOTOR_ENB_Pin;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(MOTOR_ENB_GPIO_Port, &gpio);
    }
    else if (tim_pwmHandle->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();

        gpio.Pin = MOTOR_ENA_Pin;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(MOTOR_ENA_GPIO_Port, &gpio);
    }
}

extern "C" void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *tim_pwmHandle)
{
    if (tim_pwmHandle->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_DISABLE();
    }
    else if (tim_pwmHandle->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_DISABLE();
    }
}
#endif

extern "C" void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle)
{
    if (adcHandle->Instance == ADC1)
    {
        __HAL_RCC_ADC1_CLK_DISABLE();
    }
}

extern "C" void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{
    if (uartHandle->Instance == USART2)
    {
        __HAL_RCC_USART2_CLK_DISABLE();
    }
}

static void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        Indicators_BlinkBoth(1U, 150U, 0U);
        HAL_Delay(150U);
    }
}

