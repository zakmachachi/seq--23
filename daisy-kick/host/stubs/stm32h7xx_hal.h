/* Host stub for the STM32 HAL surface midi_oled_monitor.cpp touches. */
#pragma once
#include <stdint.h>

#define __HAL_RCC_GPIOC_CLK_ENABLE() do {} while(0)
#define __HAL_RCC_USART3_CLK_ENABLE() do {} while(0)

#define GPIO_PIN_11 (1u << 11)
#define GPIO_MODE_AF_PP 2u
#define GPIO_PULLUP 1u
#define GPIO_SPEED_FREQ_VERY_HIGH 3u
#define GPIO_AF7_USART3 7u

#define UART_WORDLENGTH_8B 0u
#define UART_STOPBITS_1 0u
#define UART_PARITY_NONE 0u
#define UART_MODE_RX 1u
#define UART_HWCONTROL_NONE 0u
#define UART_OVERSAMPLING_16 0u
#define UART_ONE_BIT_SAMPLE_DISABLE 0u
#define UART_ADVFEATURE_NO_INIT 0u

#define HAL_OK 0

/* The firmware polls these; the harness pushes bytes through
 * ProcessMidiByte() directly, so RXNE never asserts here. */
#define USART_ISR_ORE       (1u << 3)
#define USART_ISR_FE        (1u << 1)
#define USART_ISR_NE        (1u << 2)
#define USART_ISR_RXNE_RXFNE (1u << 5)
#define USART_ICR_ORECF     (1u << 3)
#define USART_ICR_FECF      (1u << 1)
#define USART_ICR_NECF      (1u << 2)

typedef struct
{
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
    uint32_t Alternate;
} GPIO_InitTypeDef;

typedef struct { uint32_t dummy; } GPIO_TypeDef;
extern GPIO_TypeDef* GPIOC;

typedef struct
{
    volatile uint32_t ISR;
    volatile uint32_t ICR;
    volatile uint32_t RDR;
} USART_TypeDef;
extern USART_TypeDef* USART3;

typedef struct
{
    uint32_t BaudRate;
    uint32_t WordLength;
    uint32_t StopBits;
    uint32_t Parity;
    uint32_t Mode;
    uint32_t HwFlowCtl;
    uint32_t OverSampling;
    uint32_t OneBitSampling;
} UART_InitTypeDef;

typedef struct
{
    USART_TypeDef*  Instance;
    UART_InitTypeDef Init;
    struct { uint32_t AdvFeatureInit; } AdvancedInit;
} UART_HandleTypeDef;

inline void NVIC_SystemReset() {}
inline void HAL_GPIO_Init(GPIO_TypeDef*, GPIO_InitTypeDef*) {}
inline int  HAL_UART_Init(UART_HandleTypeDef*) { return HAL_OK; }
