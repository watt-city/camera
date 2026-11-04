# WATT City Git Guide

A quick, practical guide so 19+ people can share code and KiCad files without overwriting each other's work. If you get stuck with Git, ask in `#watt-city` on Slack!

---

## The 3 Golden Rules

### 1. Always pull before you start, push when you finish
Most Git conflicts happen because someone started editing old files.
- **Before opening KiCad or the Arduino IDE:** pull the latest (`git pull`, or **Fetch/Pull origin** in GitHub Desktop).
- **As soon as you finish a session:** commit with a clear summary (e.g. *"Tune PID Kd and add VL53L0X stop threshold"*) and push.

### 2. Only one person edits a KiCad project at a time
KiCad schematic (`.kicad_sch`) and board (`.kicad_pcb`) files **cannot be merged** if two people edit them at once.
- Before opening KiCad, post in `#watt-city`: *"Editing the Car PCB now!"*
- Pull, do your KiCad work, commit, push, and post: *"Pushed Car PCB!"*

### 3. Push straight to `main`, or make a branch?
- **Push straight to `main`** when you are:
  - updating the Python tools in `calibration/` or `tools/`
  - updating READMEs or documentation
- **Make a branch** for anything in `firmware/`, the code that drives the car:
  - Treat `firmware/` on `main` as **working code**.
  - Branch, test on the physical car, then open a Pull Request on GitHub so a lead can take a look before it merges.

## Command line

```bash
# one-time
git clone https://github.com/watt-city/<repo>.git
cd <repo>
git config user.name  "Your Name"
git config user.email "you@example.com"   # same email as your GitHub account

# every session (pushing straight to main)
git pull
# ...work...
git status                     # check what changed BEFORE adding
git add path/to/file.ino       # add the files you meant to change
git commit -m "Add VL53L0X watchdog reset"
git push

# firmware work (branch)
git pull
git checkout -b feat/espnow-receive
# ...work, test on the car...
git add firmware/...
git commit -m "Receive stoplight state over ESP-NOW"
git push -u origin feat/espnow-receive
# then open GitHub → "Compare & pull request"
```

`git add .` is fine **only after** `git status` shows nothing you didn't mean to change. The `.gitignore` already blocks KiCad backups, lock files and `secrets.h`; don't force-add them.

**Branch names:** `type/short-description`, lowercase with dashes: `feat/`, `fix/`, `hw/`, `test/`, `docs/` (e.g. `fix/pid-overshoot`).

---

## Housekeeping rules

- **Arduino folder rule:** every `.ino` file lives in a folder with the **exact same name** (e.g. `vl53l0x_test/vl53l0x_test.ino`), or the Arduino IDE won't open it.
- **No WiFi passwords or keys:** these repos are public! Put credentials in `secrets.h` (ignored by `.gitignore`) and commit a `secrets.example.h` with placeholders.
- **Large files go in the shared WATT Google Drive folder,** not Git: photos, videos, calibration image sets, datasheets.
- **Comment tuned numbers.** Pin numbers, gains and thresholds you changed on hardware get a comment saying why.
- **Log tests.** When firmware works on real hardware, put the commit hash in the hardware log so "it worked on Tuesday" can be recovered.
- **Boards sent to fab:** tag the commit (e.g. `car-hw-v1.0`) and attach the Gerber zip to a GitHub Release.

## Using AI

You must understand **exactly** what AI wrote before you commit it. Any spec AI gives you (pinouts, part numbers, voltage ratings, register values) gets checked against the datasheet first. Schematics must be your own work. Say what AI was used for in your PR.
