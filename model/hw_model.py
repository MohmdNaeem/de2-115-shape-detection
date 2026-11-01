"""Bit-exact Python reference of the FPGA shape pipeline (used to build the
template ROM and to cross-check the Verilog simulation)."""
import numpy as np, math
from shapes import shape, features as prof_features
W,H=320,240
MIN_AREA=300; MIN_DIM=16
Q=2**32
TRI_TH=int(0.0081*Q); QUAD_TH=int(0.0067*Q); REJ_TH=int(0.0105*Q)
ELONG_K=20; I1_SHIFT=10; REJECT_SCORE=700
CLASSES=['NONE','CIRCLE','SQUARE','RECTANGLE','TRIANGLE','PENTAGON','HEXAGON','ELLIPSE','UNKNOWN']
def moments_q(m):
    ys,xs=np.nonzero(m); xs=xs.astype(object); ys=ys.astype(object)
    A=len(xs); Sx=int(sum(xs)); Sy=int(sum(ys)); Sxx=int(sum(x*x for x in xs)); Syy=int(sum(y*y for y in ys)); Sxy=int(sum(x*y for x,y in zip(xs,ys)))
    return A,Sx,Sy,Sxx,Syy,Sxy
def central(A,Sx,Sy,Sxx,Syy,Sxy):
    C20=A*Sxx-Sx*Sx; C02=A*Syy-Sy*Sy; C11=A*Sxy-Sx*Sy
    A3=A*A*A
    clamp=lambda v: min(v,(1<<20)-1)
    n20=clamp((C20<<16)//A3); n02=clamp((C02<<16)//A3)
    n11m=clamp((abs(C11)<<16)//A3); n11=-n11m if C11<0 else n11m
    I1=max(0,n20*n02-n11m*n11m)
    num=(n20-n02)**2+4*n11m*n11m; den=(n20+n02)**2
    return I1,num,den
def build_templates():
    T=[]
    for r in range(0,31,5): T.append(('HEXAGON',shape('hex_flat',s=70,ang=math.radians(r))))
    for r in range(0,37,6): T.append(('PENTAGON',shape('pent_up',s=70,ang=math.radians(r))))
    T.append(('CIRCLE',shape('circle',s=70)))
    out=[]
    for c,m in T:
        f=[int(v) for v in prof_features(m)[0]]
        I1=central(*moments_q(m))[0]
        out.append((CLASSES.index(c),f,I1))
    return out
TEMPL=build_templates()
def classify_mask(m):
    """returns (class_id, bbox, debug)"""
    A=int(m.sum())
    if A<MIN_AREA: return 0,None,{'A':A}
    pf=prof_features(m); f=[int(v) for v in pf[0]]; bb=pf[2]
    Wd=bb[2]-bb[0]+1; Hd=bb[3]-bb[1]+1
    if Wd<MIN_DIM or Hd<MIN_DIM: return 0,None,{'A':A}
    I1,num,den=central(*moments_q(m))
    dbg={'A':A,'I1':I1,'num':num,'den':den,'f':f}
    elong = num*ELONG_K > den
    if I1>REJ_TH: c=8
    elif I1>TRI_TH: c=4
    elif I1>QUAD_TH: c=3 if elong else 2
    elif elong: c=7
    else:
        best=None
        for cid,t,ti in TEMPL:
            s=sum(abs(a-b) for a,b in zip(f,t)) + (abs(I1-ti)>>I1_SHIFT)
            if best is None or s<best[0]: best=(s,cid)
        dbg['score']=best[0]
        c=best[1] if best[0]<=REJECT_SCORE else 8
    return c,bb,dbg
# ---- streaming front-end model ----
def feature_val(rgb,mode):
    R=rgb[...,0].astype(int);G=rgb[...,1].astype(int);B=rgb[...,2].astype(int)
    Y=(R*77+G*150+B*29)>>8
    if mode==0: return 255-Y
    if mode==1: return Y
    return np.maximum(np.maximum(R,G),B)-np.minimum(np.minimum(R,G),B)
def isodata(f,T):
    hi=f>T; ch=int(hi.sum()); cl=f.size-ch
    sh=int(f[hi].sum()); sl=int(f[~hi].sum())
    mh=sh//ch if ch else 255; ml=sl//cl if cl else 0
    Tn=(mh+ml+1)>>1; Tn=max(16,min(240,Tn))
    return Tn, (mh-ml)>=24
def morph(b,op):
    Hh,Ww=b.shape; p=np.zeros((Hh+1,Ww+1),bool); p[1:,1:]=b
    out=np.zeros((Hh-1,Ww-1),bool)
    acc=None
    for dy in range(3):
        for dx in range(3):
            s=p[dy:dy+Hh-1,dx:dx+Ww-1]
            acc=s.copy() if acc is None else (acc&s if op=='e' else acc|s)
    return acc
def mask_from(f,T):
    b=f>T
    for op in 'edde': b=morph(b,op)
    return b
