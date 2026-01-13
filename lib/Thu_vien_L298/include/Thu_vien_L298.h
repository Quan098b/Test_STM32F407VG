#ifndef THU_VIEN_L298_H
#define THU_VIEN_L298_H

#include "stm32f4xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    L298_DIR_FWD = 0,
    L298_DIR_REV = 1
} L298_Dir_t;

typedef struct {
    GPIO_TypeDef *ENA_Port; uint8_t ENA_Pin; /* ENA = PC8 */
    GPIO_TypeDef *IN1_Port; uint8_t IN1_Pin; /* IN1 = PC13 */
    GPIO_TypeDef *IN2_Port; uint8_t IN2_Pin; /* IN2 = PE5  */
} L298_Pins_t;

typedef struct {
    L298_Pins_t pins;

    /* Soft PWM state */
    uint8_t  duty_percent;      /* 0..100 */
    uint32_t pwm_period_ms;     /* >0 */
    uint8_t  enabled;           /* 0/1 */
    uint8_t  last_ena_level;    /* cache for BSRR */
} L298_Handle_t;

/* GPIO init + set coast + ENA=0 */
void L298_Init_SoftPWM(L298_Handle_t *h, const L298_Pins_t *pins, uint32_t pwm_period_ms);

/* Set direction */
void L298_SetDir(L298_Handle_t *h, L298_Dir_t dir);

/* Coast (IN1=0, IN2=0) */
void L298_Coast(L298_Handle_t *h);

/* Enable/Disable driver (ENA still controlled by soft PWM update when enabled=1) */
void L298_Enable(L298_Handle_t *h, uint8_t en);

/* duty 0..100% */
void L298_SetDutyPercent(L298_Handle_t *h, uint8_t duty_percent);

/* Call periodically (e.g. each loop) with current ms tick */
void L298_SoftPWM_Update(L298_Handle_t *h, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
