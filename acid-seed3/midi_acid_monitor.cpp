#include "daisy_seed.h"
#include "stm32h7xx_hal.h"
#include "MidiRxQueue.h"
#include "dsp/Engine.h"
using namespace daisy;
static DaisySeed hw;
static UART_HandleTypeDef midi_uart;
static MidiRxQueue midi_rx;
static synth::Engine engine;
extern "C" void AcidMidiRxIrq(){
    if(USART3->ISR & (USART_ISR_ORE|USART_ISR_FE|USART_ISR_NE)){
        USART3->ICR=USART_ICR_ORECF|USART_ICR_FECF|USART_ICR_NECF;
        midi_rx.Discontinuity();
    }
    while(USART3->ISR & USART_ISR_RXNE_RXFNE)midi_rx.Push(uint8_t(USART3->RDR));
}
static void AudioCallback(AudioHandle::InputBuffer,AudioHandle::OutputBuffer out,size_t n){
    // Audio callback is the sole voice/control owner. IRQ only queues bytes.
    // Eight bytes per 16-frame callback still exceeds full MIDI wire rate
    // by >7x, while bounding expensive note/CC work in the audio IRQ.
    uint16_t b;unsigned budget=8;
    while(budget--&&midi_rx.Pop(b)){if(b & MidiRxQueue::GAP)engine.Discontinuity();engine.Midi(uint8_t(b));}
    for(size_t i=0;i<n;++i)engine.TickOutput(out[0][i],out[1][i]);
}
int main(){
    hw.Configure();hw.Init();
    FPU->FPDSCR |= (1UL<<24); // Also applies to fresh interrupt FP contexts.
    __set_FPSCR(__get_FPSCR() | (1UL<<24)); // Flush subnormal filter tails to zero.
    __HAL_RCC_GPIOC_CLK_ENABLE();__HAL_RCC_USART3_CLK_ENABLE();
    GPIO_InitTypeDef gpio={};gpio.Pin=GPIO_PIN_11;gpio.Mode=GPIO_MODE_AF_PP;
    gpio.Pull=GPIO_PULLUP;gpio.Speed=GPIO_SPEED_FREQ_VERY_HIGH;gpio.Alternate=GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC,&gpio);
    midi_uart.Instance=USART3;midi_uart.Init.BaudRate=31250;
    midi_uart.Init.WordLength=UART_WORDLENGTH_8B;midi_uart.Init.StopBits=UART_STOPBITS_1;
    midi_uart.Init.Parity=UART_PARITY_NONE;midi_uart.Init.Mode=UART_MODE_RX;
    midi_uart.Init.HwFlowCtl=UART_HWCONTROL_NONE;midi_uart.Init.OverSampling=UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&midi_uart)!=HAL_OK){hw.SetLed(true);while(1){}}
    HAL_UARTEx_EnableFifoMode(&midi_uart);
    USART3->ICR=USART_ICR_ORECF|USART_ICR_FECF|USART_ICR_NECF;
    HAL_NVIC_SetPriority(USART3_IRQn,0,0);
    USART3->CR1|=USART_CR1_RXNEIE_RXFNEIE;USART3->CR3|=USART_CR3_EIE;
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    hw.SetAudioBlockSize(16);hw.StartAudio(AudioCallback);
    while(1)System::Delay(10);
}
