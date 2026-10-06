"""Production MIDI/combined-bus routing and headroom checks plus listening WAVs."""
import array,itertools,json,math,pathlib,subprocess,sys,wave
binary,out=sys.argv[1],pathlib.Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
def render(**kw):
 p=out/'scratch.f32';args=dict(note=38,vel=64,shape=.5,sub=1,punch=1,mackamt=1,tubeamt=1,line=1,hits=5,tail=700,decay=.5,bpf=1,wave=.5)
 args.update(kw);subprocess.run([binary,str(p)]+[f'{k}={v}' for k,v in args.items()],check=True,capture_output=True)
 y=array.array('f');y.frombytes(p.read_bytes());p.unlink();assert all(math.isfinite(x) for x in y);return y
for name,config in [('internal',dict(externallevel=0)),('external',dict(punch=0,mackamt=0,tubeamt=0,externallevel=.5,externalhz=440))]:
 dry=render(erosion=0,**config);wet=render(erosion=1,**config)
 assert max(abs(a-b) for a,b in zip(dry,wet))>.01,name
rows=[]
for model,frequency,amount in itertools.product((0,1),(0,.5,1),(.5,1)):
 y=render(model=model,erosion=amount,erosionfreq=frequency,shape=1,wave=1,decay=1,reverb=1,bitcrush=1,externallevel=1,externalhz=55)
 peak=max(map(abs,y));assert peak<.93,peak;rows.append(dict(model=model,frequency=frequency,amount=amount,peak=peak))
for amount in (0,.5,1):
 y=render(erosion=amount,erosionfreq=.5)
 with wave.open(str(out/f'erosion-{int(amount*100)}.wav'),'wb') as w:
  w.setnchannels(1);w.setsampwidth(2);w.setframerate(48000);w.writeframes(array.array('h',(int(x*32767) for x in y)).tobytes())
(out/'headroom.json').write_text(json.dumps(rows,indent=2))
print('Both internal and external routed through effect; 12 combined-bus extreme cases; peak',max(r['peak'] for r in rows))
