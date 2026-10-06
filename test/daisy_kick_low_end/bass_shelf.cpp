#include <cstdio>
#include <cmath>
#include <complex>
#include <cassert>
#define main firmware_main
#include "../../daisy-kick/midi_oled_monitor.cpp"
#undef main
namespace daisy { uint32_t System::now_ms=0; }
static GPIO_TypeDef gp{};static USART_TypeDef ua{};
GPIO_TypeDef* GPIOC=&gp;USART_TypeDef* USART3=&ua;
int main(){puts("amount,hz,gain_db,phase_deg");for(float amount:{0.f,.25f,.5f,.75f,1.f}) {
 double previous=16;
 for(double hz:{20.,30.,40.,55.,73.416,90.,120.,180.,240.,500.,1000.,5000.,15000.}){
 CleanBassShelf shelf;shelf.Reset();std::complex<double> a{},b{};
 for(int n=0;n<96000;++n){double p=TWO_PI*hz*n/48000.;float x=sin(p), y=shelf.Process(x,amount);if(n>=48000){double w=.5-.5*cos(TWO_PI*(n-48000)/47999.);auto ph=std::polar(w,-p);a+=double(x)*ph;b+=double(y)*ph;}}
 double gain=20*log10(abs(b/a));assert(gain<15.05 && gain>-.2); if(hz<=120.) assert(gain<=previous+.02); previous=gain;
 printf("%.2f,%.3f,%.5f,%.3f\n",amount,hz,gain,arg(b/a)*180/3.141592653589793);
 }} }
