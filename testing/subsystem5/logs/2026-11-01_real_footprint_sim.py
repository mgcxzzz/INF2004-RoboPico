import math
def path(L,F,search=100,turn=45,yp=-65,step=5):
    t=math.radians(turn); poses=[]
    for d in range(0,turn+1,5): poses.append((0,yp,math.radians(d)))
    diag=L*1.414; x,y=0,yp
    for k in range(0,int(diag)+1,step): poses.append((k*math.sin(t),yp+k*math.cos(t),t))
    x,y=diag*math.sin(t),yp+diag*math.cos(t)
    for d in range(turn,-1,-5): poses.append((x,y,math.radians(d)))
    for k in range(0,int(F-L)+1,step): poses.append((x,y+k,0))
    y=y+max(0,F-L)
    for d in range(0,turn+1,5): poses.append((x,y,-math.radians(d)))
    rj=min(diag+search,600)
    for k in range(0,int(rj)+1,step): poses.append((x-k*math.sin(t),y+k*math.cos(t),-t))
    return poses
def car_pts(px,py,ph,hw=67.5,vf=130,vr=-60,n=12):
    c,s=math.cos(ph),math.sin(ph); pts=[]
    corners=[(-hw,vf),(hw,vf),(hw,vr),(-hw,vr),(-hw,vf)]
    for i in range(4):
        (u0,v0),(u1,v1)=corners[i],corners[i+1]
        for k in range(n):
            u=u0+(u1-u0)*k/n; v=v0+(v1-v0)*k/n
            pts.append((px+u*c+v*s, py-u*s+v*c))
    return pts
def dist_box(x,y,x0,x1,y0,y1):
    dx=max(x0-x,0,x-x1); dy=max(y0-y,0,y-y1); return math.hypot(dx,dy)
for stop in (150,200,250):
  for (name,x0,x1) in (('100 centred',-50,50),('180 centred',-90,90),('180 off L60',-150,30),('180 off R60',-30,150)):
    F=stop+200+190+50
    res=[]
    for side in (1,-1):
        L=317
        m=1e9
        for (px,py,ph) in path(L,F):
            px*=side; ph*=side
            for (x,y) in car_pts(px,py,ph): m=min(m,dist_box(x,y,x0,x1,stop,stop+100))
        res.append(round(m))
    print(f"stop {stop} {name:12s} min clearance real car to real box (100 deep): right {res[0]} mm, left {res[1]} mm")
