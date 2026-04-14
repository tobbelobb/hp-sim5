import sys, os
root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
sys.path.insert(0, os.path.join(root, 'src', 'python'))
sys.path.insert(0, os.path.join(root, 'example_apps', 'python', 'flipper'))
sys.path.insert(0, os.path.join(root, 'public', 'usd_scenes'))
