import json,sys
for f in sys.argv[1:]:
    d=json.loads(open(f).read().strip().splitlines()[-1]); m=d['metrics']; g=lambda k: m[k]*1000
    print(f"{d['label']:14s} score {m['score']:.0f} path rms/95/max {g('contourRms'):.2f}/{g('contourP95'):.2f}/{g('contourMax'):.2f} h/v {g('contourHorizontalRms'):.2f}/{g('contourVerticalRms'):.2f} shape {g('shapeRms'):.2f}/{g('shapeP95'):.2f} offset {[round(v*1000,2) for v in m['meanOffset']]} in.25 {m['withinFineTolerance']*100:.0f}% lag {g('meanLag'):.2f}")
