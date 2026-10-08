"""Actual firmware callback, fresh process per run. No copied audio model."""
import array,subprocess,sys,math
from pathlib import Path
binary,folder=sys.argv[1],Path(sys.argv[2])
def render(pad,external=0,hits=1):
    path=folder/'held-output.f32'
    args=dict(heldfx=pad,externallevel=external,externalhz=731,hits=hits,shape=0,
              mackamt=0,tubeamt=0,sub=1,punch=1,line=.3,tail=600,spacing_ms=300)
    subprocess.run([binary,str(path)]+[f'{k}={v}' for k,v in args.items()],check=True,capture_output=True)
    data=array.array('f');data.frombytes(path.read_bytes())
    assert all(math.isfinite(x) and abs(x)<.986 for x in data)
    return data
base=render(0)
for pad in list(range(2,8))+list(range(10,16)):
    assert render(pad)==base, f'pad {pad} changed internal kick with silent external input'
dry=render(0,.2,0)
for pad in list(range(2,9))+list(range(10,17)):
    out=render(pad,.2,0)
    assert out!=dry, f'pad {pad} did not affect external audio'
for pad in (1,9):
    assert render(pad,.2,0)==dry, f'kick-only pad {pad} changed external input'
print('PASS: pads 1/9 affect only kick; other processors preserve internal kick, except HAZARD adding its click layer to the external bus.')
