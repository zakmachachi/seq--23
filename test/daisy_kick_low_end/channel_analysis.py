#!/usr/bin/env python
"""Production DSP probes. Python 2/3 + numpy/matplotlib. BEFORE.cpp OUTPUT_DIR."""
from __future__ import print_function
import os,sys,subprocess,tempfile,json,hashlib
os.environ.setdefault('MPLCONFIGDIR','/tmp/seq-kick-channel-plots')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=os.path.abspath(os.path.join(os.path.dirname(__file__),'../..'))
out=os.path.abspath(sys.argv[2]);before=os.path.abspath(sys.argv[1])
if not os.path.exists(out):os.makedirs(out)
work=tempfile.mkdtemp(prefix='kick-channels-');bins=[];metrics={}
for name,source in [('before',before),('after',root+'/daisy-kick/midi_oled_monitor.cpp')]:
 binary=work+'/'+name;args=['c++','-std=c++17','-O2','-w','-I',root+'/daisy-kick/host/stubs','-I',root+'/daisy-kick','-DFIRMWARE="'+source+'"',root+'/test/daisy_kick_low_end/channel_probe.cpp','-o',binary]
 if name=='before':args.append('-DOLD_MODEL')
 subprocess.check_call(args);bins.append(binary)
 with open(source,'rb') as f:metrics[name+'_source_sha256']=hashlib.sha256(f.read()).hexdigest()
eqfile=out+'/eq.csv';subprocess.check_call([bins[1],'eq',eqfile]);eq=np.loadtxt(eqfile,delimiter=',')
center=eq[eq[:,3]==1];metrics['eq_max_center_gain_error_db']=float(np.max(abs(center[:,4]-15)))
assert metrics['eq_max_center_gain_error_db']<.03
fig,axes=plt.subplots(2,1,figsize=(11,8))
for rate in [48000,192000]:
 e=center[center[:,2]==rate];axes[0].plot(e[:,0],e[:,1],label='%d Hz processing rate'%rate)
axes[0].set_yscale('log');axes[0].set_xlabel('MIDI CC44/45/46 value');axes[0].set_ylabel('Actual EQ centre (Hz)');axes[0].legend();axes[0].grid(True)
for cc in [0,32,64,96,127]:
 e=eq[(eq[:,0]==cc)&(eq[:,2]==192000)];axes[1].semilogx(e[:,1]*e[:,3],e[:,4],'o-',label='CC %d'%cc)
axes[1].set_xlabel('Frequency (Hz)');axes[1].set_ylabel('EQ gain (dB)');axes[1].legend();axes[1].grid(True);fig.tight_layout();fig.savefig(out+'/mid_eq.png',dpi=140)
alias=[];fig,axes=plt.subplots(2,2,figsize=(13,8))
for row,hz in enumerate([997,3001]):
 for col,count in enumerate([0,3]):
  for idx,name in enumerate(['before','after']):
   path=work+'/tone.f32';subprocess.check_call([bins[idx],'alias',path,str(count),str(hz)])
   x=np.fromfile(path,dtype='<f4').astype(float);e=abs(np.fft.rfft(x))**2;freq=np.fft.rfftfreq(len(x),1./48000)
   harm=np.zeros(len(e),dtype=bool)
   for k in range(1,int(24000/hz)+1):harm|=abs(freq-k*hz)<=2
   valid=(freq>=20)&(freq<=20000)
   adb=float(10*np.log10(e[valid&~harm].sum()/e[valid&harm].sum()))
   alias.append(dict(version=name,hz=hz,layers=count,nonharmonic_to_harmonic_db=adb,peak=float(max(abs(x)))))
   axes[row,col].plot(freq,10*np.log10(np.maximum(e/e.max(),1e-15)),label=name,alpha=.7)
  axes[row,col].set_title('%d Hz, %d mid/drive stages'%(hz,count));axes[row,col].set_xlim(0,20000);axes[row,col].set_ylim(-120,5);axes[row,col].grid(True);axes[row,col].legend()
fig.tight_layout();fig.savefig(out+'/aliasing.png',dpi=140);metrics['alias_probes']=alias
path=work+'/glue.f32';subprocess.check_call([bins[1],'glue',path]);g=np.fromfile(path,dtype='<f4').reshape(-1,3).astype(float)
gain=20*np.log10(g[:,2]);metrics['glue_hot_sub']={'maximum_reduction_db':float(-gain.min()),'gain_ripple_last_500ms_db':float(np.ptp(gain[-24000:])),'first_5ms_reduction_db':float(-gain[12000:12240].min())}
fig,ax=plt.subplots(figsize=(11,3));ax.plot(np.arange(len(g))/48.,gain);ax.set_xlabel('Time (ms), 36.7 Hz sine starts at 250 ms');ax.set_ylabel('Gain (dB)');ax.grid(True);fig.tight_layout();fig.savefig(out+'/glue.png',dpi=140)
# Actual hit trains: output 2 observes mixer2 before glue, with no DSP changes.
source_path=root+'/daisy-kick/midi_oled_monitor.cpp'
with open(source_path) as f:source=f.read()
marker='out[EXTERNAL_OUTPUT_CHANNEL][i] = mixed_output;'
assert source.count(marker)==1
observed=work+'/observed.cpp'
with open(observed,'w') as f:f.write(source.replace(marker,'out[EXTERNAL_OUTPUT_CHANNEL][i] = (kick_output+external_output)*MIX_OUTPUT_TRIM;'))
with open(root+'/daisy-kick/host/kick_host.cpp') as f:host=f.read()
with open(work+'/host.cpp','w') as f:f.write(host.replace('#include "../midi_oled_monitor.cpp"','#include "'+observed+'"'))
renderer=work+'/render'
subprocess.check_call(['c++','-std=c++17','-O2','-w','-I',root+'/daisy-kick/host/stubs','-I',root+'/daisy-kick',work+'/host.cpp','-o',renderer])
trains=[];fig,axes=plt.subplots(3,1,figsize=(11,7))
for ax,(bpm,external) in zip(axes,[(185,0),(250,0),(250,.8)]):
 raw=work+'/train.f32';pre=work+'/pre.f32';spacing=60000./bpm
 args=dict(note=38,vel=64,tmod=0,shape=.5,decay=.75,line=1,punch=1,sub=1,mackie=1,mackamt=1,model=0,bpf=1,layers=1,bpf1=.15,bpf2=.5,bpf3=.85,wave=1,hits=8,spacing_ms=spacing,tail=800,externallevel=external,externalhz=60,externalout=pre)
 subprocess.check_call([renderer,raw]+['%s=%s'%item for item in sorted(args.items())])
 post=np.fromfile(raw,dtype='<f4').astype(float);preaudio=np.fromfile(pre,dtype='<f4').astype(float)
 assert max(abs(post))<.93
 valid=abs(preaudio)>1e-5;gain=np.ones(len(post));gain[valid]=post[valid]/preaudio[valid]
 gd=20*np.log10(np.maximum(gain,1e-12));onsets=[]
 # Harness rounds each render span down to the sixteen-sample block size.
 period=960+int((spacing-20)*48)//16*16
 for hit in range(8):
  start=12000+hit*period;onsets.append(float(-gd[start:start+240].min()))
 entry=dict(bpm=bpm,external_level=external,max_reduction_db=float(-gd.min()),max_first_5ms_reduction_db=max(onsets),peak=float(max(abs(post))))
 trains.append(entry);ax.plot(np.arange(len(post))/48000.,gd);ax.set_title('%d BPM, external amplitude %.1f'%(bpm,external));ax.set_ylabel('Glue gain (dB)');ax.set_ylim(-2.1,.1);ax.grid(True)
axes[-1].set_xlabel('Time (seconds)');fig.tight_layout();fig.savefig(out+'/glue_kick_trains.png',dpi=140);metrics['glue_kick_trains']=trains
pitchfile=out+'/tail_pitch.csv';subprocess.check_call([bins[1],'pitch',pitchfile]);pitch=np.loadtxt(pitchfile,delimiter=',')
fig,axes=plt.subplots(2,1,figsize=(11,7))
for ax,rate in zip(axes,[0,64]):
 for vel in [0,32,64,96,127]:
  p=pitch[(pitch[:,0]==vel)&(pitch[:,1]==rate)];ax.plot(p[:,2],12*np.log2(p[:,3]/73.41619),label='Velocity %d'%vel)
 ax.set_title('TMOD %d: %s'%(rate,'single glide' if rate==0 else 'resettable 1.414 Hz LFO'));ax.set_ylim(-13,30);ax.set_ylabel('Semitones from D2');ax.grid(True);ax.legend(loc='best',fontsize=8)
axes[-1].set_xlabel('Time after trigger (ms)');fig.tight_layout();fig.savefig(out+'/tail_pitch.png',dpi=140)
with open(out+'/channel_metrics.json','w') as f:json.dump(metrics,f,indent=2)
print(json.dumps(metrics,indent=2));print('Probe workspace:',work)
