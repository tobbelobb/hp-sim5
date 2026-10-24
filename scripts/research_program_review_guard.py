"""Keep a separate coding-client review finite even if its tool host crashes."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    deadline, marker, *argv = sys.argv[1:]
    if float(deadline) <= time.time():
        Path(marker).write_text('Review wall allocation expired before client startup\n')
        return 124
    try:
        process = subprocess.Popen(argv)
    except OSError as error:
        print(str(error), file=sys.stderr)
        return 127
    try:
        return process.wait(timeout=max(.001, float(deadline) - time.time()))
    except subprocess.TimeoutExpired:
        Path(marker).write_text('Review wall allocation expired\n')
        # The controller spawned this guard as its own session/group. Only
        # descendants of this review share it, never a producer or supervisor.
        os.killpg(os.getpgrp(), signal.SIGKILL)


if __name__ == '__main__':
    sys.exit(main())
