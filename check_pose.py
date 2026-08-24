#!/usr/bin/env python3
"""
Validate xiao_intrinsics.json against physical reality.

RMS reprojection error only tells you the model fits the images you fed it.
It says nothing about whether the resulting distances are correct. These two
checks do:

  board mode  - solvePnP on the checkerboard. Isolates intrinsics + --square.
  tag mode    - AprilTag pose. Adds --tag-size to the chain.

Run board mode FIRST. If board mode is off by a consistent percentage, the
problem is --square, not your intrinsics, and tag mode will inherit it.

    pip install opencv-python numpy requests
    pip install pupil-apriltags          # tag mode only

    python check_pose.py --intrinsics xiao_intrinsics.json --mode board \\
        --square 18.3 --url http://192.168.1.42/capture --expect 300

    python check_pose.py --intrinsics xiao_intrinsics.json --mode tag \\
        --tag-size 52 --url http://192.168.1.42/capture --expect 500

Measure --expect from the FRONT OF THE LENS with a tape. The optical centre
sits a few mm behind that, so expect to read a few mm long at short range.
That offset is constant; a percentage error is not.
"""

import argparse
import json
import sys

import cv2
import numpy as np
import requests


def load_intrinsics(path):
    with open(path) as f:
        d = json.load(f)
    K = np.array([[d["fx"], 0.0, d["cx"]],
                  [0.0, d["fy"], d["cy"]],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    dist = np.array(d["dist_coeffs"], dtype=np.float64)
    size = (d["resolution"]["width"], d["resolution"]["height"])
    return K, dist, size, d


def grab(url):
    r = requests.get(url, timeout=5)
    r.raise_for_status()
    arr = np.frombuffer(r.content, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError("undecodable frame")
    return img


def verdict(measured_mm, expected_mm):
    if expected_mm is None:
        return
    err = measured_mm - expected_mm
    pct = 100.0 * err / expected_mm
    print(f"  expected      {expected_mm:8.1f} mm")
    print(f"  error         {err:+8.1f} mm  ({pct:+.1f}%)")
    print()
    if abs(pct) < 2.0:
        print("  PASS - within 2%.")
    elif abs(pct) < 5.0:
        print("  MARGINAL - 2-5% off. Re-measure the tape distance and re-run a")
        print("  few times. If it is consistent, suspect the scale input.")
    else:
        print(f"  FAIL - {pct:+.1f}% off. This is a scale error, not noise.")
        print("  A scale input is wrong by roughly the same percentage.")
        print(f"  Implied correct value = stated_size / {1 + pct/100:.4f}")


def board_mode(args, K, dist, calib_size):
    img = grab(args.url)
    h, w = img.shape[:2]
    if (w, h) != calib_size:
        sys.exit(f"Frame is {w}x{h} but intrinsics are for "
                 f"{calib_size[0]}x{calib_size[1]}. These do not apply.")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    pattern = (args.cols, args.rows)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not ok:
        sys.exit("No board found. Fill more of the frame and hold still.")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    corners = cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), criteria)

    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square

    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, dist,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        sys.exit("solvePnP failed.")

    t = tvec.ravel()
    dist_mm = float(np.linalg.norm(t))
    print("\nBOARD POSE")
    print(f"  square size   {args.square:8.2f} mm  (input)")
    print(f"  t             [{t[0]:8.1f} {t[1]:8.1f} {t[2]:8.1f}] mm")
    print(f"  Z (forward)   {t[2]:8.1f} mm")
    print(f"  |t|           {dist_mm:8.1f} mm")
    verdict(float(t[2]), args.expect)


def tag_mode(args, K, dist, calib_size):
    try:
        from pupil_apriltags import Detector
    except ImportError:
        sys.exit("pip install pupil-apriltags")

    img = grab(args.url)
    h, w = img.shape[:2]
    if (w, h) != calib_size:
        sys.exit(f"Frame is {w}x{h} but intrinsics are for "
                 f"{calib_size[0]}x{calib_size[1]}. These do not apply.")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # pupil-apriltags has no distortion model, so undistort first and hand it
    # the pinhole intrinsics that match the undistorted image.
    newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 0, (w, h))
    gray = cv2.undistort(gray, K, dist, None, newK)
    fx, fy = newK[0, 0], newK[1, 1]
    cx, cy = newK[0, 2], newK[1, 2]

    det = Detector(families="tag36h11", nthreads=2,
                   quad_decimate=1.0, decode_sharpening=0.25)

    # pupil-apriltags wants METRES; everything else here is mm.
    results = det.detect(gray, estimate_tag_pose=True,
                         camera_params=(fx, fy, cx, cy),
                         tag_size=args.tag_size / 1000.0)
    if not results:
        sys.exit("No tag detected. Move closer, add light, or check the family "
                 "is tag36h11.")

    print("\nTAG POSE")
    print(f"  tag size      {args.tag_size:8.2f} mm  (input, black-square outer edge)")

    for r in results:
        t = r.pose_t.ravel() * 1000.0   # back to mm
        c = r.corners
        px_w = float(max(np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)))
        print()
        print(f"  ID {r.tag_id}")
        print(f"    pixel width {px_w:8.1f} px   (under ~25 px = unreliable decode)")
        print(f"    t           [{t[0]:8.1f} {t[1]:8.1f} {t[2]:8.1f}] mm")
        print(f"    Z (forward) {t[2]:8.1f} mm")
        print(f"    decision margin {r.decision_margin:.1f}")

    if len(results) == 1:
        print()
        verdict(float(results[0].pose_t.ravel()[2] * 1000.0), args.expect)
    else:
        print("\n  Multiple tags in frame; skipping the pass/fail check.")
        print("  Re-run with a single tag visible.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--intrinsics", default="xiao_intrinsics.json")
    ap.add_argument("--url", required=True, help="e.g. http://192.168.1.42/capture")
    ap.add_argument("--mode", choices=["board", "tag"], required=True)
    ap.add_argument("--expect", type=float, default=None,
                    help="tape-measured distance in mm for the pass/fail check")
    # board mode
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square", type=float, default=18.3, help="mm")
    # tag mode
    ap.add_argument("--tag-size", type=float, default=52.0,
                    help="mm, OUTER EDGE OF THE BLACK SQUARE (not the white tile)")
    args = ap.parse_args()

    K, dist, calib_size, raw = load_intrinsics(args.intrinsics)
    print(f"Intrinsics: {args.intrinsics}")
    print(f"  {calib_size[0]}x{calib_size[1]}  fx={raw['fx']:.2f} fy={raw['fy']:.2f} "
          f"cx={raw['cx']:.2f} cy={raw['cy']:.2f}")
    print(f"  RMS {raw['rms_reprojection_error_px']:.4f} px "
          f"from {raw['n_images']} images")

    if args.mode == "board":
        board_mode(args, K, dist, calib_size)
    else:
        tag_mode(args, K, dist, calib_size)


if __name__ == "__main__":
    sys.exit(main())
