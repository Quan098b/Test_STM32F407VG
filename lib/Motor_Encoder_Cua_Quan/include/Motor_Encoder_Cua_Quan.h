#ifndef MOTOR_ENCODER_CUA_QUAN_H
#define MOTOR_ENCODER_CUA_QUAN_H

#include "stm32f4xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    GPIO_TypeDef *gpioA;  /* kênh A */
    uint8_t       pinA;   /* 0..15 */
    GPIO_TypeDef *gpioB;  /* kênh B */
    uint8_t       pinB;   /* 0..15 */

    volatile int32_t count;     /* tổng count (có dấu) */
    volatile uint8_t prev_ab;   /* trạng thái trước (B<<1)|A */
} EncoderEXTI_CQ_Handle;

/* Init encoder EXTI quadrature (both edges), pull-up, LUT decode */
void EncoderEXTI_CQ_Init(EncoderEXTI_CQ_Handle *h);

/* Gọi trong EXTI9_5_IRQHandler (PC7/PC9 thuộc nhóm 5..9) */
void EncoderEXTI_CQ_IRQHandler(EncoderEXTI_CQ_Handle *h);

/* Get total count (atomic) */
int32_t EncoderEXTI_CQ_GetCount(EncoderEXTI_CQ_Handle *h);

/* Reset count về 0 (atomic) */
void EncoderEXTI_CQ_Reset(EncoderEXTI_CQ_Handle *h);

#ifdef __cplusplus
}
#endif

#endif
