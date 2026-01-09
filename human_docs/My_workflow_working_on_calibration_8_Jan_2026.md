I open a terminal with Ctrl-Alt-t.
I cd to the hp-sim5 repo with `cd sim`.
I type `npx vite` to get the repo wide server running.
I move that terminal to another work area so it doesn't interfere with my Alt-Tab later.
I open my browser with Super-1.
I open up the webpage with this long and complicated url: http://localhost:5173/hp-sim5/hp-sim/?gcode_ws=ws://localhost:8790. I usually find it by typing `ws` in the
address bar, and it then shows up in the suggestions.
I open up another terminal with Ctrl-Alt-t and maximize it with Super-Up.
I type `cd sim` again, to get to the hp-sim5 repo again.
I do Ctrl-Shift-t a few times to get a few tabs open in my new shell, all with work dir in hp-sim5.

I change some code.
I do a test run. Currently often like this:
```
python autocal/active_calibrate.py ellipse-loop --work-dataset autocal/data/test.json --collector-args --speedup 32
```


