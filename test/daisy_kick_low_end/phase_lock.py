#!/usr/bin/env python
"""Plot actual oscillator phase clocks before/after; Python 2.7 or 3 + numpy/matplotlib.
Usage: phase_lock.py BEFORE_FIRMWARE.cpp OUTPUT_DIR
The before source is supplied explicitly; no duplicate synthesis model.
"""
from __future__ import print_function
import os,sys,tempfile,subprocess,json,hashlib
os.environ.setdefault('MPLCONFIGDIR','/tmp/seq-kick-phase-matplotlib')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=os.path.abspath(os.path.join(os.path.dirname(__file__),'../..'))
output=os.path.abspath(sys.argv[2])
if not os.path.exists(output): os.makedirs(output)
work=tempfile.mkdtemp(prefix='kick-phase-clocks-')
code=r'''
#include <cstdio>
#include <initializer_list>
#define main firmware_main
#include FIRMWARE
#undef main
namespace daisy { uint32_t System::now_ms=0; }
static GPIO_TypeDef gpio{}; static USART_TypeDef uart{};
GPIO_TypeDef* GPIOC=&gpio; USART_TypeDef* USART3=&uart;
int main(){
 for(float shape : {.25f,.5f,1.f}){
  KickVoice v; v.Trigger(73.41619f,shape,.55f,.55f,100,0,.3f,0,64,64);
  for(int i=0;i<12000;++i){
   float before=v.phase;
   float bodyHz=v.base_hz*(1.f+v.pitch_depth*v.pitch_env);
   float error=v.body_phase-2.f*v.phase; error-=roundf(error);
   KickVoiceOut o; v.Process(o);
   float step=v.phase-before; if(step<0)step+=1.f;
   if(i%24==0)printf("%g,%g,%g,%g,%g\n",shape,i/48.f,bodyHz*.5f,step*48000.f,error*360.f);
  }
 }
}
'''
with open(os.path.join(work,'probe.cpp'),'w') as f:f.write(code)
fig,axes=plt.subplots(3,2,figsize=(12,9))
provenance={}
for index,(label,source) in enumerate([('Previous upload',os.path.abspath(sys.argv[1])),('Shared clock',os.path.join(root,'daisy-kick/midi_oled_monitor.cpp'))]):
 binary=os.path.join(work,'probe'+str(index));raw=os.path.join(output,'clock-'+str(index)+'.csv')
 subprocess.check_call(['c++','-std=c++17','-O2','-w','-I',os.path.join(root,'daisy-kick/host/stubs'),'-I',os.path.join(root,'daisy-kick'),'-DFIRMWARE="'+source+'"',os.path.join(work,'probe.cpp'),'-o',binary])
 with open(raw,'wb') as f:subprocess.check_call([binary],stdout=f)
 data=np.loadtxt(raw,delimiter=',')
 with open(source,'rb') as f: provenance[label]=hashlib.sha256(f.read()).hexdigest()
 for row,shape in enumerate([.25,.5,1.]):
  d=data[abs(data[:,0]-shape)<.001]
  axes[row,0].plot(d[:,1],d[:,3],label=label+' sub',color=['#dc2626','#2563eb'][index])
  if index==1: axes[row,0].plot(d[:,1],d[:,2],'--',color='#111827',label='New body frequency / 2')
  axes[row,1].plot(d[:,1],d[:,4],label=label,color=['#dc2626','#2563eb'][index])
for row,shape in enumerate([.25,.5,1.]):
 axes[row,0].set_title('D2, SHAPE %d%%: octave frequency' % (shape*100));axes[row,0].set_ylabel('Hz');axes[row,0].set_ylim(0,400)
 axes[row,1].set_title('Body phase minus twice sub phase');axes[row,1].set_ylabel('Wrapped phase error (degrees)');axes[row,1].set_ylim(-185,185)
 for a in axes[row]:a.grid(True,alpha=.25);a.set_xlabel('Time from trigger (ms)');a.legend(fontsize=8);a.set_xlim(0,250)
fig.tight_layout();fig.savefig(os.path.join(output,'oscillator_phase.png'),dpi=150);fig.savefig(os.path.join(output,'oscillator_phase.svg'))
with open(os.path.join(output,'phase-provenance.json'),'w') as f:json.dump(provenance,f,indent=2)
print('Saved actual generator phase traces to',output)
