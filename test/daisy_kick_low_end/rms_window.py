from __future__ import print_function
import os,json,wave
os.environ.setdefault('MPLCONFIGDIR','/tmp/seq-kick-ripple-matplotlib')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root='daisy-kick/analysis';hz=440.*2.**((38.-69.)/12)/2;sr=48000
fig,axs=plt.subplots(2,1,figsize=(11,7));metrics={}
for ax,folder,label in zip(axs,['coherent','locked'],['Previous uploaded clean D2 + SUB','New shared-clock clean D2 + SUB']):
 w=wave.open(root+'/'+folder+'/d2_clean_sub.wav','rb');x=np.frombuffer(w.readframes(w.getnframes()),dtype='<i2').astype(float)/32768.;w.close();x=x[4800:]
 n=960;cs=np.r_[0,np.cumsum(x*x)];ii=np.arange(0,min(len(x)-n,24000),48);r=np.sqrt((cs[ii+n]-cs[ii])/n);tt=(ii+n*.5)/sr
 ax.plot(tt*1000,20*np.log10(np.maximum(r,1e-12)),label='20 ms sliding RMS',alpha=.7)
 # Consecutive one-sub-cycle energy windows at a fixed phase of the settled tone.
 edges=np.rint((.15+np.arange(14)/hz)*sr).astype(int);t=[];rr=[]
 for a,b in zip(edges[:-1],edges[1:]):
  t.append((a+b)*.5/sr);rr.append(np.sqrt(np.mean(x[a:b]**2)))
 t=np.array(t);lev=20*np.log10(rr);trend=np.polyval(np.polyfit(t,lev,1),t)
 select=(tt>=.15)&(tt<=.48);short=20*np.log10(r[select]);shortres=short-np.polyval(np.polyfit(tt[select],short,1),tt[select])
 metrics[folder]={'sub_hz':hz,'sub_cycle_ms':1000/hz,'20ms_detrended_ripple_peak_to_peak_db':float(np.ptp(shortres)),'cycle_rms_detrended_ripple_peak_to_peak_db':float(np.ptp(lev-trend)),'source':'exported PCM16 actual firmware audition, first 100 ms silence removed'}
 ax.plot(t*1000,lev,'o-',label='Consecutive complete sub cycles',color='black',markersize=4)
 ax.set_title(label);ax.set_xlim(100,500);ax.set_ylim(-65,-15);ax.set_ylabel('RMS dBFS');ax.set_xlabel('Time since trigger (ms)');ax.grid(True,alpha=.25);ax.legend(loc='best')
fig.tight_layout();fig.savefig(root+'/locked/rms_window_comparison.png',dpi=150);fig.savefig(root+'/locked/rms_window_comparison.svg')
with open(root+'/locked/rms_window_metrics.json','w') as f:json.dump(metrics,f,indent=2)
print(json.dumps(metrics,indent=2))
