# Signed contour error (mm, + = outside the commanded square) for every extruding tip sample.
import json,sys,numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
def load(fn):
    d=json.load(open(fn))
    tip=np.array([x if x else [np.nan]*3 for x in d['tip']])*1000
    return d,tip
def contour(tip):
    x,y=tip[:,0],tip[:,1]
    cheb=np.maximum(np.abs(x),np.abs(y))
    h=np.clip(np.round(cheb/100)*100,200,600)
    return h, cheb-h   # Chebyshev radius minus square half-size = signed normal distance to the side
if __name__=='__main__':
    files=sys.argv[1:]
    fig,axs=plt.subplots(1,2,figsize=(14,6.5))
    for fn in files:
        d,tip=load(fn); h,e=contour(tip); ok=~np.isnan(e)
        # magnified XY: draw (commanded point) + 20*error along normal
        x,y=tip[ok,0],tip[ok,1]; c=np.maximum(np.abs(x),np.abs(y)); s=(c+20*e[ok])/c
        axs[0].plot(x*s,y*s,'.',ms=0.6,label=fn)
        print(fn, 'all-sample RMS %.3f mm, p99 |e| %.3f, max %.3f'%(np.sqrt(np.nanmean(e**2)),np.nanpercentile(np.abs(e),99),np.nanmax(np.abs(e))))
        for H in (200,300,400,500,600):
            for cx,cy,name in ((1,1,'+x+y'),(1,-1,'+x-y'),(-1,-1,'-x-y'),(-1,1,'-x+y')):
                m=ok&(h==H)&(np.hypot(tip[:,0]-cx*H,tip[:,1]-cy*H)<40)
                print('  h=%d corner %s  max+ %.2f  max- %.2f'%(H,name,np.nanmax(e[m]),np.nanmin(e[m])))
    axs[0].set_aspect('equal'); axs[0].set_title('tip path, contour error x20'); axs[0].legend(markerscale=10)
    d,tip=load(files[0]); h,e=contour(tip)
    axs[1].plot(np.array(d['t']),e,lw=0.5); axs[1].set_xlabel('t (s)'); axs[1].set_ylabel('contour error mm (+out)')
    fig.tight_layout(); fig.savefig('contour.png',dpi=110)
