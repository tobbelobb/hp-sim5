# Contour error of the deposited tip vs the commanded squares, per side, near corners.
import json,sys,numpy as np
d=json.load(open(sys.argv[1]))
tip=np.array([x if x else [np.nan]*3 for x in d['tip']])*1000  # mm
t=np.array(d['t'])
ext=~np.isnan(tip[:,0])
out=[]
for h in (200,300,400,500,600):
    for name,axis,val,other in (('+x side',0,h,1),('-y side',1,-h,0),('-x side',0,-h,1),('+y side',1,h,0)):
        m=ext&(np.abs(tip[:,axis]-val)<15)&(np.abs(tip[:,other])<=h+1)
        if not m.any(): continue
        dev=tip[m,axis]-val  # signed normal deviation
        s=tip[m,other]
        k=np.argmax(np.abs(dev))
        out.append((h,name,m.sum(),np.abs(dev).max(),s[k],dev[k],np.median(dev)))
for o in out: print("h=%d %-8s n=%5d max|dev|=%.3f mm at s=%.1f (dev %.3f) median %.3f"%o)
