# Setup and test troubleshooting on Ubuntu 24.04

This note is for the common "first `npx jest` after an Ubuntu upgrade" situation in `hp-sim5`.

## Recommended Node setup

Use a current **LTS** Node release for day-to-day repo work.

```bash
nvm install 24
nvm use 24
nvm alias default 24
node -v
npm install
```

Odd-numbered short-lived Node releases can work, but they are not the recommended baseline for local development.

## When the problem is environment setup, not application code

A failing `npx jest` run does **not** automatically mean missing dependencies or broken code.

Typical setup/runtime failures:

- `ENOSPC: System limit for number of file watchers reached`
- Vite startup failure
- Puppeteer or browser-backed tests failing later with `ERR_CONNECTION_REFUSED`

If Vite crashes first, browser tests usually fail as a consequence.

## Linux inotify fix

If you see `ENOSPC`, increase Linux's inotify limits:

```bash
sudo sysctl fs.inotify.max_queued_events=16384
sudo sysctl fs.inotify.max_user_instances=8192
sudo sysctl fs.inotify.max_user_watches=524288
```

Make it persistent across reboots:

```bash
printf 'fs.inotify.max_queued_events=16384\nfs.inotify.max_user_instances=8192\nfs.inotify.max_user_watches=524288\n' | sudo tee /etc/sysctl.d/99-inotify.conf
sudo sysctl --system
```

Useful diagnostics:

```bash
cat /proc/sys/fs/inotify/max_user_watches
cat /proc/sys/fs/inotify/max_user_instances
ulimit -Sn
```

## Vite watch scope

`hp-sim5` already ignores some heavy directories in `vite.config.js`, including `RRF/**`.

If you add more generated directories locally, keep them out of the watch set too. Common examples:

- `.venv/`
- large build outputs
- cache directories
- object-file trees

## How to interpret remaining Jest failures

After Node/npm and inotify are fixed, rerun the tests.

What is left after that is more likely to be a **real code/test mismatch**, for example:

- a unit test expecting an old force/tension command value
- a geometry helper returning the wrong value for an inside-sphere case
- a markup test whose expectation is too broad or stale

So the practical order is:

1. use Node LTS
2. run `npm install`
3. fix inotify if needed
4. rerun `npx jest`
5. only then debug the remaining red tests as application/test issues
