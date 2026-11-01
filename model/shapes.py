import numpy as np, math
W,H=320,240
def poly_mask(pts):
    yy,xx=np.mgrid[0:H,0:W]; px=xx+0.5; py=yy+0.5
    inside=np.zeros((H,W),bool); n=len(pts)
    for i in range(n):
        x1,y1=pts[i]; x2,y2=pts[(i+1)%n]
        cond=((y1>py)!=(y2>py)) & (px < (x2-x1)*(py-y1)/(y2-y1+1e-12)+x1)
        inside^=cond
    return inside
def ellipse_mask(cx,cy,rx,ry,ang=0):
    yy,xx=np.mgrid[0:H,0:W]; dx=xx+0.5-cx; dy=yy+0.5-cy
    c,s=math.cos(ang),math.sin(ang); u=dx*c+dy*s; v=-dx*s+dy*c
    return (u/rx)**2+(v/ry)**2<=1
def rot(pts,cx,cy,ang):
    c,s=math.cos(ang),math.sin(ang)
    return [(cx+(x-cx)*c-(y-cy)*s, cy+(x-cx)*s+(y-cy)*c) for x,y in pts]
def regular(n,cx,cy,r,ang0):
    return [(cx+r*math.cos(ang0+2*math.pi*i/n), cy+r*math.sin(ang0+2*math.pi*i/n)) for i in range(n)]
def shape(name,cx=160,cy=120,s=60,ang=0.0,aspect=1.0):
    d=math.radians
    if name=='circle': return ellipse_mask(cx,cy,s,s,ang)
    if name=='ellipse': return ellipse_mask(cx,cy,s*aspect,s,ang)
    if name=='square': p=[(cx-s,cy-s),(cx+s,cy-s),(cx+s,cy+s),(cx-s,cy+s)]
    if name=='rect': p=[(cx-s*aspect,cy-s),(cx+s*aspect,cy-s),(cx+s*aspect,cy+s),(cx-s*aspect,cy+s)]
    if name=='diamond': p=[(cx,cy-s*1.2),(cx+s*1.2,cy),(cx,cy+s*1.2),(cx-s*1.2,cy)]
    if name=='tri_up': p=regular(3,cx,cy+s*0.2,s*1.2,d(-90))
    if name=='tri_down': p=regular(3,cx,cy-s*0.2,s*1.2,d(90))
    if name=='tri_left': p=regular(3,cx,cy,s*1.2,d(180))
    if name=='tri_right': p=regular(3,cx,cy,s*1.2,d(0))
    if name=='tri_rbl': p=[(cx-s,cy-s),(cx-s,cy+s),(cx+s,cy+s)]
    if name=='tri_rbr': p=[(cx+s,cy-s),(cx-s,cy+s),(cx+s,cy+s)]
    if name=='tri_rtl': p=[(cx-s,cy-s),(cx+s,cy-s),(cx-s,cy+s)]
    if name=='tri_rtr': p=[(cx-s,cy-s),(cx+s,cy-s),(cx+s,cy+s)]
    if name=='pent_up': p=regular(5,cx,cy,s*1.1,d(-90))
    if name=='pent_down': p=regular(5,cx,cy,s*1.1,d(90))
    if name=='hex_flat': p=regular(6,cx,cy,s*1.1,d(0))
    if name=='hex_point': p=regular(6,cx,cy,s*1.1,d(90))
    if name=='star': 
        p=[]
        for i in range(10):
            r=s*1.2 if i%2==0 else s*1.2*0.382
            a=d(-90)+math.pi*i/5; p.append((cx+r*math.cos(a),cy+r*math.sin(a)))
    if name=='octagon': p=regular(8,cx,cy,s*1.1,d(22.5))
    return poly_mask(rot(p,cx,cy,ang))

def features(m):
    ys,xs=np.nonzero(m)
    if len(xs)==0: return None
    xmin,xmax,ymin,ymax=xs.min(),xs.max(),ys.min(),ys.max()
    Wd=xmax-xmin+1; Hd=ymax-ymin+1
    rc=m.sum(1); cc=m.sum(0)
    f=[]
    for k in range(1,8):
        y=ymin+((Hd*k)>>3); s3=int(rc[y-1]+rc[y]+rc[y+1])
        f.append(min(255,(s3*85)//Wd))
    for k in range(1,8):
        x=xmin+((Wd*k)>>3); s3=int(cc[x-1]+cc[x]+cc[x+1])
        f.append(min(255,(s3*85)//Hd))
    asp=min(255,(Wd*64)//Hd)
    return f,asp,(xmin,ymin,xmax,ymax),int(m.sum())
