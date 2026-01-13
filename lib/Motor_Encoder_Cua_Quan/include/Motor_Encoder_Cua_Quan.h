#ifndef MOTOR_ENCODER_CUA_QUAN_H
#define MOTOR_ENCODER_CUA_QUAN_H

#include "stm32f4xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== ENCODER (EXTI quadrature) ===================== */
typedef struct {
    GPIO_TypeDef *gpioA; uint8_t pinA;   /* C1 */
    GPIO_TypeDef *gpioB; uint8_t pinB;   /* C2 */
    volatile int32_t count;
    uint8_t last_ab;                     /* 2-bit state */
} EncoderEXTI_CQ_Handle;

void     EncoderEXTI_CQ_Init(EncoderEXTI_CQ_Handle *h);
int16_t  EncoderEXTI_CQ_GetDelta(EncoderEXTI_CQ_Handle *h);
int32_t  EncoderEXTI_CQ_GetCount(EncoderEXTI_CQ_Handle *h);
void     EncoderEXTI_CQ_Reset(EncoderEXTI_CQ_Handle *h);

/* Gọi trong ISR EXTI9_5_IRQHandler (vì PC7 và PC9 đều nằm trong EXTI9_5) */
void     EncoderEXTI_CQ_IRQHandler(EncoderEXTI_CQ_Handle *h);


/* ===================== MOTOR L298 (GPIO: ENA/IN1/IN2) ===================== */
typedef struct {
    GPIO_TypeDef *gpio_ena; uint8_t pin_ena;   /* ENA: enable (ON/OFF) */
    GPIO_TypeDef *gpio_in1; uint8_t pin_in1;
    GPIO_TypeDef *gpio_in2; uint8_t pin_in2;
} MotorL298_CQ_Handle;

void MotorL298_CQ_Init(MotorL298_CQ_Handle *m);
void MotorL298_CQ_Enable(MotorL298_CQ_Handle *m, uint8_t en); /* ENA=1/0 */
void MotorL298_CQ_SetDir(MotorL298_CQ_Handle *m, int8_t dir); /* +1/-1/0 */

#ifdef __cplusplus
}
#endif
#endif
