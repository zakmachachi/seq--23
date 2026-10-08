#pragma once
#include <stdint.h>
namespace midiwire {
inline bool IsSync(uint8_t b){return b==0xf8||b==0xfa||b==0xfb||b==0xfc;}
// Called inside a saved/restored interrupt guard. CCs leave room for one
// full layered note burst; every channel packet leaves three realtime slots.
template<class Port> bool Message(Port& port,uint8_t status,uint8_t a,uint8_t b,bool control){
    if(port.availableForWrite()<(control?124:6))return false;
    port.write(status);port.write(a&127);port.write(b&127);return true;
}
}
