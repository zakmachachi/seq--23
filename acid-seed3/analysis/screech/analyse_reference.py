import wave,array,cmath,math,struct,zlib,json,sys
from pathlib import Path
w=wave.open(sys.argv[1],'rb');sr=w.getframerate();duration=w.getnframes()/sr
assert w.getnchannels()==2 and w.getsampwidth()==2, "Requires stereo 16-bit PCM WAV"
n=2048
rev=[int(format(i,'011b')[::-1],2) for i in range(n)]
roots=[cmath.exp(-2j*math.pi*i/n) for i in range(n//2)]
def fft(x):
 a=[complex(x[i]) for i in rev];m=2
 while m<=n:
  h=m//2
  for k in range(0,n,m):
   for j in range(h):
    t=roots[j*n//m]*a[k+j+h];u=a[k+j];a[k+j]=u+t;a[k+j+h]=u-t
  m*=2
 return a
cols=[];stats=[]
for t in range(int(duration*3)):
 w.setpos(min(int(t*sr/3),w.getnframes()-n));a=array.array('h',w.readframes(n));x=[(a[2*i]+a[2*i+1])/65536*(.5-.5*math.cos(2*math.pi*i/(n-1))) for i in range(n)]
 z=fft(x);mag=[abs(v)/n for v in z[:n//2]]
 vals=[]
 for j in range(256):
  hz=120*pow(12000/120,j/255);k=round(hz*n/sr);db=20*math.log10(max(mag[max(1,k-1):k+2])+1e-8);vals.append(max(0,min(1,(db+65)/50)))
 cols.append(vals)
 if t%24==0:
  k=max(range(25,400),key=lambda k:mag[k]);stats.append({'seconds':t/3,'strongest_mid_hz':round(k*sr/n),'mid_db':round(20*math.log10(mag[k]+1e-8),1)})
width=len(cols);data=b''.join(b'\0'+bytes(v for c in cols for v in (int(c[y]*255),int(c[y]**2*230),int(c[y]**.5*170))) for y in reversed(range(256)))
def chunk(k,d):return struct.pack('>I',len(d))+k+d+struct.pack('>I',zlib.crc32(k+d))
Path(__file__).with_name('screech-reference.png').write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',width,256,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(data))+chunk(b'IEND',b''))
Path(__file__).with_name('screech-reference.json').write_text(json.dumps({'duration':duration,'rate':sr,'snapshots':stats},indent=2));print(duration,stats)
