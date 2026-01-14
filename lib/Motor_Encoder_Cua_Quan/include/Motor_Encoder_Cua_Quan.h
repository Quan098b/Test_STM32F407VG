#ifndef MOTOR_ENCODER_CUA_QUAN_H
#define MOTOR_ENCODER_CUA_QUAN_H

#include "stm32f4xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
  Thư viện encoder phiên bản ổn định:
  - Dùng TIM3 Encoder Interface (hardware quadrature)
  - Chân:
      PC6 = TIM3_CH1 (AF2)
      PC7 = TIM3_CH2 (AF2)
  - TIM3->CNT là 16-bit; mở rộng lên count32 bằng EncoderEXTI_CQ_Task().
*/

/* Giữ tên type cũ để main.c ít phải sửa */
typedef struct
{
    TIM_TypeDef *tim;            /* TIM3 */
    volatile int32_t count32;    /* mở rộng 32-bit */
    volatile uint16_t prev_cnt;  /* CNT 16-bit lần trước */
} EncoderEXTI_CQ_Handle;

/* Init encoder TIM3 PC6/PC7 */
void EncoderEXTI_CQ_Init(EncoderEXTI_CQ_Handle *h);

/* Gọi định kỳ trong main loop để cập nhật count32 */
void EncoderEXTI_CQ_Task(EncoderEXTI_CQ_Handle *h);

/* Get total count (atomic) */
int32_t EncoderEXTI_CQ_GetCount(EncoderEXTI_CQ_Handle *h);

/* Reset count về 0 (atomic) */
void EncoderEXTI_CQ_Reset(EncoderEXTI_CQ_Handle *h);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_ENCODER_CUA_QUAN_H */
