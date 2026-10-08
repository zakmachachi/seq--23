#pragma once
#include "Blocks.h"
namespace heldfx {
struct HazardFx {
    // 5.33 ms speaker click from a real Eberline E-120, CC0, cookies+policy:
    // https://freesound.org/people/cookies%2Bpolicy/sounds/555919/
    // Trimmed, DC-centred, normalized and edge-windowed; no pitched oscillator.
    static float Click(unsigned i){
        static const int16_t sample[256]={
            0,1,-1,9,-9,19,-7,15,6,-12,57,-42,61,-24,20,19,
            -41,97,-107,142,-87,97,15,1957,14863,29176,32700,31682,27868,16491,891,-11069,
            -19300,-26011,-31400,-31946,-20847,-10651,-11818,-14151,-16272,-17111,-12252,-7220,-3647,1034,3726,1027,
            -3731,-6190,-5947,-3864,-350,5665,11783,14161,15536,17486,16267,12687,9600,6097,3827,3634,
            2434,478,-1117,-2784,-3245,-2168,-1665,-1984,-2017,-2488,-3490,-4518,-5961,-7679,-9264,-10247,
            -10158,-8995,-6355,-2244,1372,3649,5767,7630,8364,7863,6684,5047,3375,2805,3275,3784,
            3580,2825,1392,-894,-2778,-3733,-2591,1489,5139,5607,5106,5152,3373,242,-2113,-3859,
            -3881,-2419,-1348,-545,-66,-802,-1866,-1516,-822,-1158,-1783,-1664,-871,-854,-1190,-1097,
            -1408,-1711,-1731,-1840,-1919,-1694,-1160,-366,987,1930,1874,1679,1242,473,-168,-512,
            -496,-377,-386,-294,-136,-573,-1628,-2466,-2686,-2204,-1272,-498,283,1106,1733,2413,
            2637,2276,2181,2345,2177,1575,1079,715,183,-186,-284,-97,196,138,137,646,
            1403,2207,2387,1005,-901,-1654,-1348,-1117,-1655,-1545,366,1416,171,-343,417,15,
            -687,-512,-169,578,1294,1461,1723,1718,779,3,435,832,499,227,0,-307,
            -868,-1730,-2297,-2759,-3304,-3075,-2018,-1321,-1225,-1100,-685,65,721,818,582,325,
            236,353,510,517,344,206,213,235,151,10,-19,158,396,425,196,-44,
            -114,-52,23,61,111,162,177,162,114,51,-8,-45,-39,-19,-5,0,
        };
        return sample[i]*(1.f/32768.f);
    }
    Random rng;
    unsigned age[4]={256,256,256,256},voice=0,refractory=0;
    float amplitude[4]={},probability=0;
    void Reset(){rng.Reset();for(auto& a:age)a=256;voice=refractory=0;probability=0;}
    void Tune(const Macro& m,const Analysis& input){
        float density=2.f+180.f*m.character*m.character*(.35f+.65f*Unit(input.slow*10.f));
        probability=density/SR;
    }
    float Process(float x,const Macro&,const Analysis&,float){
        if(refractory)--refractory;
        if(!refractory && rng.Uniform()<probability){
            age[voice]=0;amplitude[voice]=.32f+.06f*rng.Uniform();
            voice=(voice+1)&3;refractory=72;
        }
        float clicks=0;
        for(unsigned i=0;i<4;++i)if(age[i]<256)clicks+=Click(age[i]++)*amplitude[i];
        return x+clicks;
    }
};
}
