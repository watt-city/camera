#!/usr/bin/env python3
"""Keep every sketch's copy of camera_cfg.h identical to firmware/shared/camera_cfg.h.

Arduino can only #include headers that sit in the sketch's own folder, so each
sketch that needs camera_cfg.h holds a copy. Edit ONLY firmware/shared/camera_cfg.h,
then run:

    python tools/sync_camera_cfg.py          # copy shared -> every sketch
    python tools/sync_camera_cfg.py --check  # just report copies that differ
"""
import pathlib
import shutil
import sys

root = pathlib.Path(__file__).resolve().parent.parent
src = root / "firmware" / "shared" / "camera_cfg.h"
copies = [p for p in (root / "firmware").glob("*/camera_cfg.h") if p.parent.name != "shared"]

stale = [p for p in copies if p.read_bytes() != src.read_bytes()]
if "--check" in sys.argv:
    for p in stale:
        print(f"OUT OF SYNC: {p.relative_to(root)}")
    sys.exit(1 if stale else 0)
for p in stale:
    shutil.copyfile(src, p)
    print(f"updated {p.relative_to(root)}")
print(f"{len(copies)} copies checked, {len(stale)} updated")
