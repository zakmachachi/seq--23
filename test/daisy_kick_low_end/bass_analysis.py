import os, subprocess, json, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
out=sys.argv[3] if len(sys.argv)>3 else 'daisy-kick/analysis/bass-shelf'
binaries={'before':sys.argv[1], 'after':sys.argv[2]}
if not os.path.isdir(out): os.makedirs(out)
def render(which, **kw):
 p='/tmp/kick-bass-shelf/'+which+'.f32'
 settings=dict(note=38,vel=64,tmod=0,sub=0,punch=1,shape=0,decay=1,line=.15,mackamt=0,tubeamt=0,wave=0,hits=1,tail=650)
 settings.update(kw)
 subprocess.check_call([binaries[which],p]+['%s=%s'%i for i in settings.items()],stdout=open(os.devnull,'w'),stderr=open(os.devnull,'w'))
 return np.fromfile(p,dtype=np.float32)[12000:]
rows=[]
for shape in (.1,.25,.5,.75,1):
 for sweep in (0,.25,.5,1):
  for dirty in (0,1):
   vals=[]
   for version in ('before','after'):
    y=render(version,shape=shape,sweeptime=sweep,mackamt=dirty,bpf=dirty,layers=1)
    # Remove the intentional -6 dB output-trim difference for this attack comparison.
    if version=='after': y=y*2
    z=y[:960]*np.hanning(960)
    spec=np.abs(np.fft.rfft(z))**2
    hz=np.fft.rfftfreq(len(z),1./48000)
    vals.append(float(spec[hz>=5000].sum()))
   rows.append(dict(shape=shape,sweep=sweep,dirty=dirty,hf_change_db=float(10*np.log10(max(vals[1],1e-30)/max(vals[0],1e-30)))))
fig,axes=plt.subplots(2,1,figsize=(10,7))
for boost in (0,.5,1):
 y=render('after',sub=boost)
 n=32768
 spec=20*np.log10(np.maximum(2*np.abs(np.fft.rfft(y[:n]*np.hanning(n)))/np.hanning(n).sum(),1e-9))
 hz=np.fft.rfftfreq(n,1./48000)
 axes[0].semilogx(hz,spec,label='Bass %d%%'%(boost*100))
 axes[1].plot(np.arange(4800)/48.,y[:4800],label='Bass %d%%'%(boost*100))
axes[0].set_xlim(20,1000); axes[0].set_ylim(-110,-25); axes[0].set_ylabel('Hann-windowed spectrum (dBFS)');axes[0].grid();axes[0].legend()
axes[1].set_xlabel('Time (ms)');axes[1].set_ylabel('Output amplitude');axes[1].legend();axes[1].grid()
fig.tight_layout();fig.savefig(out+'/d2-bass.png',dpi=150)
json.dump(dict(attack_comparisons=rows,normalization='After multiplied by 2 to remove -6.02 dB trim difference; first20ms Hann FFT energy above5kHz'),open(out+'/attack.json','w'),indent=2)
print('Attack HF changes min/max dB',min(r['hf_change_db'] for r in rows),max(r['hf_change_db'] for r in rows))
