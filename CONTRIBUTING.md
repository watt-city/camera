# Contributing to WATT City

Thanks for building WATT City! These rules keep 19+ people from overwriting each other's work. If anything here is unclear, ask in `#watt-city` on Slack.

## The short version

1. Never commit directly to `main`. It's protected, and pushes will be rejected.
2. Make a branch, do your work, push, and open a **pull request (PR)**.
3. Get **one approval** from a lead or reviewer, then merge.
4. Log every hardware test in the hardware log, and link it in your PR.

## One-time setup

```bash
git clone https://github.com/watt-city/<repo>.git
cd <repo>
git config user.name  "Your Name"
git config user.email "you@example.com"   # same email as your GitHub account
```

## Every time you work

```bash
git checkout main
git pull                              # start from the latest main
git checkout -b feat/espnow-receive   # new branch for this task

# ...edit, test on hardware...

git status                            # check what changed
git add path/to/file.ino              # add specific files, not "git add ." blindly
git commit -m "Add ESP-NOW receiver for stoplight state"
git push -u origin feat/espnow-receive
```

Then open GitHub, click **Compare & pull request**, fill in the template, and request a reviewer.

If `main` moves while you're working, update your branch with `git pull origin main`.

### Branch names

`type/short-description`, lowercase with dashes:

| Prefix | Use for | Example |
|---|---|---|
| `feat/` | new functionality | `feat/pedestrian-button` |
| `fix/` | bug fixes | `fix/pid-overshoot` |
| `hw/` | KiCad / PCB changes | `hw/stoplight-rev-b` |
| `test/` | test sketches, calibration | `test/qtr-glossy-tape` |
| `docs/` | READMEs, notes | `docs/bringup-steps` |

Keep branches short-lived (days, not weeks) and focused on one thing.

### Commit messages

Write a short, imperative summary that says **what** changed: "Fix QTR threshold for glossy tape," not "updates" or "stuff." Commit small and often.

## Pull requests

- Fill in every section of the PR template, including **how you tested** and **what AI was used for**.
- Reviewers check that the code makes sense, was tested, and doesn't break someone else's work. Respond to comments, push fixes to the same branch, and the PR updates automatically.
- Merge with **"Create a merge commit"** or **"Squash and merge"**. Delete your branch after merging.

## Hardware (KiCad) rules

- **Only one person edits a KiCad project at a time.** Schematic and PCB files don't merge, so a conflict means redoing someone's work by hand. Open a GitHub issue saying you're editing the board, and close it when your PR merges.
- Don't commit KiCad backups, lock files, or `.kicad_prl`. The `.gitignore` handles this; don't force-add them.
- When a board goes to fab: tag the commit (e.g. `stoplight-hw-v2.0`) and attach the Gerber zip to a GitHub Release.

## Firmware rules

- **Never commit WiFi passwords, API keys, or tokens. These repos are public.** Put them in `secrets.h` (gitignored) and commit a `secrets.example.h` with placeholders.
- Each Arduino sketch lives in a folder with the **same name** as its `.ino` file.
- Comment your code, especially pin numbers and anything you tuned on hardware.
- When firmware works on real hardware, note the commit hash in the hardware log so "it worked on Tuesday" can be recovered.

## Large files

Photos, videos, calibration image sets, and datasheets go in the **WATT shared folder**, not git. Link them from your PR or the hardware log.

## Using AI

You must understand **exactly** what AI wrote before you commit it. Any spec AI gives you (pinouts, part numbers, voltage ratings, register values) gets checked against the datasheet first. Schematics must be your own work. Say what AI was used for in your PR.
