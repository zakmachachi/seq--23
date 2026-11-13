#define main firmware_main
#include "../midi_oled_monitor.cpp"
#undef main
namespace daisy {uint32_t System::now_ms=0;}
static GPIO_TypeDef gpioStorage{};
static USART_TypeDef uartStorage{};
GPIO_TypeDef* GPIOC=&gpioStorage;
USART_TypeDef* USART3=&uartStorage;
#include <cassert>
#include <cstdio>
int main(){AddedClockedDelay d;d.Reset();PERF_CLOCKED_DELAY_ENABLED=true;macro_fx_value_delay=1;
float prev=0,delta=0,peak=0;
for(unsigned i=0;i<192000;++i){if(i==140000)PERF_CLOCKED_DELAY_ENABLED=false;if(i==170000)PERF_CLOCKED_DELAY_ENABLED=true;
float q=i<96000?333.3333f:400.f;float input=.2f*sinf(6.2831853f*997*i/48000.f);float y=d.Process(input,q);assert(std::isfinite(y));peak=fmaxf(peak,fabsf(y));delta=fmaxf(delta,fabsf(y-prev));prev=y;}
printf("Kick external delay: tempo jump and off/on, max step %.6f, peak %.5f\n",delta,peak);assert(delta<.1f);}
