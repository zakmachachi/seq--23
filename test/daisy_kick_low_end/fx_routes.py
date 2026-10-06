"""Audible routing on actual callback output, fresh firmware process per render."""
import array, math, subprocess, sys
from pathlib import Path
binary, folder = sys.argv[1], Path(sys.argv[2])

def render(**kw):
    path=folder/'fx-routing.f32'
    args=dict(note=38, vel=64, shape=.6, decay=.6, sub=1, punch=1,
              mackamt=.8, mackie=1, bpf=1, layers=1, bpf1=.6,
              line=.3, hits=2, spacing_ms=300, tail=650, fxroutes=0)
    args.update(kw)
    subprocess.run([binary,str(path)]+[f'{k}={v}' for k,v in args.items()],check=True,capture_output=True)
    out=array.array('f');out.frombytes(path.read_bytes())
    assert all(math.isfinite(v) and abs(v)<=.9851 for v in out)
    return out

base=render()
for name in ('delay','loop','stut','hpf','lpf'):
    assert render(**{name:1},fxroutes=15)==base, f'{name} leaked into kick'
for bit,name in enumerate(('pump','reverb','bitcrush','erosion')):
    assert render(**{name:1})==base, f'EXT-only {name} changed kick'
    both=render(**{name:1},fxroutes=1<<bit)
    rms=math.sqrt(sum((x-y)**2 for x,y in zip(base,both))/len(base))
    assert rms>1e-5, f'EXT+INT {name} did not affect kick'
    # With no generated kick, routing bit cannot change the external path.
    ext_args=dict(hits=0,tail=900,externalhz=731,externallevel=.2,**{name:.75})
    a=render(**ext_args,fxroutes=0);b=render(**ext_args,fxroutes=1<<bit)
    assert a==b, f'{name}: external path changed when adding internal route'
    if name!='pump': # Pump needs the kick/clock sidechain key.
        ext_dry=render(hits=0,tail=900,externalhz=731,externallevel=.2)
        assert a!=ext_dry, f'{name}: external effect inaudible'
    print(f'{name}: EXT isolation exact, INT difference RMS {rms:.6f}, external route invariant')
print('PASS: five EXT-only effects and four independently routed effects on actual firmware output')

# Internal-only reverb: exact external bypass, full kick reverb retained.
ext=dict(hits=0,tail=900,externalhz=731,externallevel=.2)
assert render(**ext,reverb=1,fxroutes=18)==render(**ext,reverb=0,fxroutes=18)
assert render(reverb=1,fxroutes=18)==render(reverb=1,fxroutes=2)
assert render(reverb=1,fxroutes=18)!=base
print('PASS: INT-only reverb leaves external audio exact and retains internal reverb')
