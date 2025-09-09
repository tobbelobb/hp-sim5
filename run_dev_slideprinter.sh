#
#!/bin/bash

# Path to your conda.sh (adjust if different)
CONDA_SH="$HOME/mambaforge/etc/profile.d/conda.sh"

# Terminal 1: Run `npx vite`
gnome-terminal -- bash -c "npx vite; exec bash"

# Terminal 2: Activate conda and run server
gnome-terminal -- bash -c "source $CONDA_SH && conda activate env_isaaclab && python -m examples.python.slideprinter.server; exec bash"

# Terminal 3: Activate conda and print move_commander command
gnome-terminal -- bash -c "source $CONDA_SH && conda activate env_isaaclab && sleep 0.5 && python -m examples.python.slideprinter.move_commander; exec bash"

brave-browser "http://localhost:5173/hp-sim5/examples/python/slideprinter/index.html" &
