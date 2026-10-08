"""Run with Python + numpy/scipy/matplotlib (system Python 2.7 supported).
Reference preview is CC0; comparisons are level-matched, not sample-matched.
"""
from __future__ import print_function
import os, wave, json, hashlib
import numpy as np
import scipy.signal as sig
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=os.path.dirname(os.path.abspath(__file__))
def read(name):
    w=wave.open(os.path.join(ROOT,name));sr=w.getframerate()
    x=np.frombuffer(w.readframes(w.getnframes()),'<i2').reshape(-1,w.getnchannels()).astype(float)/32768.
    return sr,x
files=['reference/chameon-hoover.wav','before-held.wav','after-held.wav']
titles=['Downloaded gabber Hoover: Chameon (CC0)','Previous engine: held D3','Revised engine: held D3']
fig,axes=plt.subplots(3,2,figsize=(13,10));parts=[];metrics={}
for row,name in enumerate(files):
    sr,x=read(name);mono=x.mean(axis=1);rms=np.sqrt(np.mean(x*x));peak=np.max(np.abs(x))
    block=int(sr*.01);env=np.array([np.sqrt(np.mean(x[i:i+block]**2)) for i in range(0,len(x)-block,block)])
    axes[row,0].plot(np.arange(len(env))*.01,20*np.log10(env+1e-8));axes[row,0].set_ylim(-65,0);axes[row,0].set_xlim(0,2.1);axes[row,0].set_ylabel('RMS dBFS');axes[row,0].set_title(titles[row]);axes[row,0].set_xlabel('Seconds')
    p,f,t=matplotlib.mlab.specgram(mono/(rms+1e-12)*.12,Fs=sr,NFFT=2048,noverlap=1920)
    axes[row,1].pcolormesh(t,f,10*np.log10(p+1e-15),cmap='hot',vmin=-85,vmax=-20)
    axes[row,1].set_yscale('log');axes[row,1].set_ylim(50,6000);axes[row,1].set_xlim(0,2.1);axes[row,1].set_ylabel('Hz');axes[row,1].set_xlabel('Seconds');axes[row,1].set_title('Level-matched spectral movement')
    y=x*.12/rms
    if sr!=48000:
        positions=np.arange(int(round(len(y)*48000./sr)))*sr/48000.
        y=np.column_stack([np.interp(positions,np.arange(len(y)),y[:,ch]) for ch in range(2)])
    assert np.max(np.abs(y))<1
    parts.extend([y,np.zeros((12000,2))])
    metrics[name]={'sample_rate':sr,'duration':len(x)/float(sr),'peak_dbfs':float(20*np.log10(peak)),'rms_dbfs':float(20*np.log10(rms)),'sha256':hashlib.sha256(open(os.path.join(ROOT,name),'rb').read()).hexdigest()}
fig.tight_layout();fig.savefig(os.path.join(ROOT,'comparison.png'),dpi=140);plt.close(fig)
combined=np.concatenate(parts);w=wave.open(os.path.join(ROOT,'reference-before-after.wav'),'wb');w.setnchannels(2);w.setsampwidth(2);w.setframerate(48000);w.writeframes((combined*32767).astype('<i2').tostring());w.close()
fig,ax=plt.subplots(figsize=(10,4))
for name in ['before','after']:
    a=np.genfromtxt(os.path.join(ROOT,name+'-control.csv'),delimiter=',',skip_header=1)
    st=12*np.log2(a[:,1]/146.83238)
    ax.plot(a[:,0],st,label=name)
    mask=(a[:,0]>.005)&(a[:,0]<1.8);indices=np.where(mask)[0];peakindex=indices[np.argmax(st[mask])]
    metrics[name+'_pitch']={'peak_semitones':float(st[peakindex]),'peak_time_s':float(a[peakindex,0]),'at_100ms_semitones':float(st[100]),'at_500ms_semitones':float(st[500]),'at_1s_semitones':float(st[1000])}
ax.set_xlim(0,2.05);ax.set_ylim(-22,16);ax.set_xlabel('Seconds');ax.set_ylabel('DCO semitones relative to held D3');ax.axhline(0,color='gray',linewidth=.6);ax.axvline(1.84,color='gray',linestyle='--');ax.legend(loc='upper right');ax.set_title('Actual oscillator control trajectory; dashed line = note-off');fig.tight_layout();fig.savefig(os.path.join(ROOT,'pitch-contours.png'),dpi=140);plt.close(fig)
with open(os.path.join(ROOT,'metrics.json'),'w') as f:json.dump(metrics,f,indent=2,sort_keys=True)
print(json.dumps(metrics,indent=2,sort_keys=True))
