#ifndef THU_VIEN_L298_H
#define THU_VIEN_L298_H

#include "stm32f4xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    L298_DIR_STOP = 0,
    L298_DIR_FWD  = 1,
    L298_DIR_REV  = -1
} L298_Dir_t;

typedef enum {
    L298_ENA_GPIO = 0,   /* ENA là GPIO bật/tắt */
    L298_ENA_PWM  = 1    /* ENA là PWM từ Timer */
} L298_EnaMode_t;

/* Cấu hình chân */
typedef struct {
    GPIO_TypeDef *ENA_Port; uint8_t ENA_Pin; /* ENA */
    GPIO_TypeDef *IN1_Port; uint8_t IN1_Pin; /* IN1 */
    GPIO_TypeDef *IN2_Port; uint8_t IN2_Pin; /* IN2 */
} L298_Pins_t;

/* PWM config (chỉ dùng khi ENA_PWM) */
typedef struct {
    TIM_TypeDef *TIMx;
    uint8_t      CH;      /* 1..4 */
    GPIO_TypeDef *PWM_Port;
    uint8_t      PWM_Pin;
    uint8_t      PWM_AF;  /* Alternate Function number */
    uint32_t     PWM_Hz;  /* tần số PWM */
} L298_PWM_t;

typedef struct {
    L298_EnaMode_t mode;
    L298_Pins_t pins;
    L298_PWM_t  pwm;
} L298_Handle_t;

/* ===== API ===== */
void L298_Init_GPIO(L298_Handle_t *h, const L298_Pins_t *pins);        /* ENA GPIO */
void L298_Init_PWM (L298_Handle_t *h, const L298_Pins_t *pins,
                    const L298_PWM_t *pwm);                            /* ENA PWM */

void L298_SetDir   (L298_Handle_t *h, L298_Dir_t dir);
void L298_Enable   (L298_Handle_t *h, uint8_t en);                     /* có tác dụng cho cả 2 mode */
void L298_SetDuty  (L298_Handle_t *h, uint8_t duty_percent);           /* chỉ có tác dụng khi ENA_PWM */

#ifdef __cplusplus
}
#endif

#endif /* THU_VIEN_L298_H */
