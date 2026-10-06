#include <cassert>
#include <cmath>
#include <cstdio>
#include "../../daisy-kick/erosion_fx.h"
int main(){
 ErosionFx e;float maxStep=0,last=0;
 for(int n=0;n<48000;++n){float x=.7f*sinf(n*.1f);assert(e.Process(x,0,.5f)==x);}
 e.Reset();for(int n=0;n<480000;++n)assert(e.Process(0,1,(n%48000)/48000.f)==0);
 for(int n=0;n<48000*30;++n){float x=.7f*sinf(n*.037f);float y=e.Process(x,(n/12000)%2,(n/24000)%2);
 assert(std::isfinite(y)&&fabsf(y)<=.700001f);assert(std::isfinite(e.ic1)&&std::isfinite(e.ic2));
 maxStep=fmaxf(maxStep,fabsf(y-last));last=y;}
 for(int n=0;n<48000;++n)e.Process(0,0,1);
 assert(e.Process(.5f,0,1)==.5f);assert(fabsf(e.frequency-12000)<1);
 double difference=0;e.Reset();for(int n=0;n<48000;++n){float x=.7f*sinf(n*.2f);float y=e.Process(x,1,.5f);difference+=(y-x)*(y-x);}
 assert(difference>10);assert(fabsf(ErosionFx::Frequency(0)-80)<.01);
 printf("Erosion: exact bypass/silence; 30s control stress finite and peak bounded; audible modulation energy %.3f; max stressed sample step %.3f\n",difference,maxStep);
}
