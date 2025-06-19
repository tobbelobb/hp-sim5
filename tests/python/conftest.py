import sys, os
root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
sys.path.insert(0, os.path.join(root, 'src', 'python'))
sys.path.insert(0, os.path.join(root, 'examples', 'python', 'flipper'))
sys.path.insert(0, os.path.join(root, 'examples', 'usd_scenes'))
