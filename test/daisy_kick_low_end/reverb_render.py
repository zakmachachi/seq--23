"""Render the real firmware and check full-bus reverb headroom. Stdlib only."""
import array,itertools,json,math,pathlib,subprocess,sys,wave
binary,out=sys.argv[1],pathlib.Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
def render(name,**kw):
    path=out/(name+'.f32')
    args=dict(note=38,vel=64,shape=.5,sweeptime=.4,decay=.45,sub=1,punch=1,
              mackamt=1,bpf=1,layers=1,line=1,wave=.4,tmod=0,bpm=180,
              spacing_ms=333.333,hits=6,tail=4000)
    args.update(kw)
    subprocess.run([binary,str(path)]+[f'{k}={v}' for k,v in args.items()],check=True,capture_output=True)
    y=array.array('f');y.frombytes(path.read_bytes());path.unlink()
    assert all(math.isfinite(v) for v in y)
    return y[12000:]
metrics=[]
for model,amount,note,external,shape in itertools.product((0,1),(.5,1),(28,38,47),(0,1),(0,.5,1)):
    y=render('check',model=model,reverb=amount,note=note,externallevel=external,externalhz=55,
             wave=1,shape=shape,decay=1,bitcrush=1,tubeamt=1,mackie=1,tube=1,bpf1=0,bpf2=0,bpf3=0)
    peak=max(map(abs,y));assert peak<.93,(model,amount,note,external,peak)
    metrics.append(dict(model=model,amount=amount,note=note,external=external,shape=shape,peak=peak))
for amount in (0,.5,.75,1):
    y=render('listen',reverb=amount)
    pcm=array.array('h',(int(max(-1,min(1,v))*32767) for v in y))
    with wave.open(str(out/f'reverb-{int(amount*100)}.wav'),'wb') as w:
        w.setnchannels(1);w.setsampwidth(2);w.setframerate(48000);w.writeframes(pcm.tobytes())
(out/'headroom.json').write_text(json.dumps(metrics,indent=2))
print(f'{len(metrics)} full-bus extreme cases: max peak {max(m["peak"] for m in metrics):.6f}; no safety ceiling reached; four listening WAVs saved.')
