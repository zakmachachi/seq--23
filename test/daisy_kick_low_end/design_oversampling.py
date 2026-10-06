#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import os
n=np.arange(97)-48
h=2.*21000/192000*np.sinc(2.*21000/192000*n)*np.kaiser(97,8.)
h/=h.sum()
root=os.path.abspath(os.path.join(os.path.dirname(__file__),'../..'))
with open(os.path.join(root,'daisy-kick/mackie_fir.h'),'w') as f:
 f.write('// Generated 97-tap Kaiser FIR: 192 kHz, 21 kHz cutoff, beta=8.\n#pragma once\nstatic constexpr float MACKIE_FIR[97] = {\n')
 for i in range(0,97,4):f.write('    '+', '.join('%.12ef'%v for v in h[i:i+4])+',\n')
 f.write('};\n')
