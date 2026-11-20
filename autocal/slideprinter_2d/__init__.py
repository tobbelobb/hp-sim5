"""Legacy 2D slideprinter autocal: the original ellipse calibration.

Only the slideprinter, spool searches on it, and retired projected-angle datasets
use this package. Each sweep holds one line, so the effector moves on a circle in
the plane and the drive and sensor lengths trace an ellipse that this code fits,
with its own filtering and spool-search machinery. Hangprinters use the point fit
(pose_fit.py, spool_fit.py) and the ellipse reference fit (ellipse_fit.py) instead.
"""
