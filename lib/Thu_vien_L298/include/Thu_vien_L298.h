#ifndef THU_VIEN_L298_H
#define THU_VIEN_L298_H

#include "stm32f4xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    GPIO_TypeDef *ENA_Port; uint8_t ENA_Pin;   /* ENA (GPIO output) */
    GPIO_TypeDef *IN1_Port; uint8_t IN1_Pin;   /* IN1 */
    GPIO_TypeDef *IN2_Port; uint8_t IN2_Pin;   /* IN2 */
} L298_Pins_t;

typedef enum {
    L298_DIR_FWD = 0,
    L298_DIR_REV = 1
} L298_Dir_t;

typedef enum {
    L298_STOP_COAST = 0,
    L298_STOP_BRAKE = 1
} L298_StopMode_t;

typedef struct {
    L298_Pins_t pins;

    /* Soft PWM for ENA */
    uint32_t pwm_period_ms;     /* e.g. 2..10ms */
    uint8_t  duty_percent;      /* 0..100 */

    /* internal state */
    uint8_t  enabled;           /* 0/1 */
    uint8_t  last_ena_level;    /* 0/1 */
} L298_Handle_t;

/* Init GPIO + soft PWM parameters (ENA must be a normal GPIO pin) */
void L298_Init(L298_Handle_t *h, const L298_Pins_t *pins, uint32_t pwm_period_ms);

/* Enable/Disable (keeps direction pins unchanged) */
void L298_Enable(L298_Handle_t *h, uint8_t en);

/* Direction control */
void L298_SetDir(L298_Handle_t *h, L298_Dir_t dir);

/* Output mode */
void L298_Coast(L298_Handle_t *h);  /* IN1=0, IN2=0 */
void L298_Brake(L298_Handle_t *h);  /* IN1=1, IN2=1 (electrical braking) */

/* Speed set */
void L298_SetDutyPercent(L298_Handle_t *h, uint8_t duty_percent); /* 0..100 */
void L298_SetSpeed8(L298_Handle_t *h, uint8_t speed8);            /* 0..255 -> 0..100 */

/* Must be called frequently in main loop to generate soft PWM on ENA */
void L298_Task(L298_Handle_t *h, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* THU_VIEN_L298_H */
