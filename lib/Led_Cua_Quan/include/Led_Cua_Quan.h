#ifndef LED_CUA_QUAN_H
#define LED_CUA_QUAN_H

#include <stdint.h>
#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LED_CQ_ACTIVE_HIGH = 0,  // SET -> sáng
    LED_CQ_ACTIVE_LOW  = 1   // RESET -> sáng
} Led_CQ_ActiveLevel;

/**
 * @brief Khởi tạo LED tại PC7 (GPIOC Pin 7) theo mức tác động.
 * @param level LED_CQ_ACTIVE_HIGH hoặc LED_CQ_ACTIVE_LOW
 */
void Led_CQ_Init(Led_CQ_ActiveLevel level);

/** Bật LED */
void Led_CQ_On(void);

/** Tắt LED */
void Led_CQ_Off(void);

/** Đảo trạng thái LED */
void Led_CQ_Toggle(void);

/**
 * @brief Nhấp nháy LED theo delay dạng vòng lặp (không dùng SysTick).
 * @param cycles số vòng lặp trễ (tăng -> nháy chậm)
 */
void Led_CQ_BlinkDelay(volatile uint32_t cycles);

#ifdef __cplusplus
}
#endif

#endif /* LED_CUA_QUAN_H */
