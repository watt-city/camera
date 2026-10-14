#!/usr/bin/env python3
"""
Intrinsic calibration for the XIAO ESP32S3 Sense (OV2640).

Usage:
    pip install opencv-python numpy
    python calibrate_xiao.py "captures/*.jpg" --cols 9 --rows 6 --square 25.0

Notes:
  - --cols/--rows are INNER corners, not squares. A board printed with
    10x7 squares has 9x6 inner corners.
  - --square is the physical square edge length in mm. Measure the PRINTED
    board with calipers; printers scale.
  - Every image must come from the exact framesize + lens focus you will
    run AprilTag detection at. Mixing resolutions silently ruins the fit.
"""

import argparse
import glob
import json
import sys

import cv2
import numpy as np


def find_corners(paths, pattern, refine_win=(5, 5)):
    """Detect chessboard corners in each image. Returns (img_points, used_paths, size)."""
    img_points = []
    used = []
    size = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

    for p in paths:
        img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if img is None:
            print(f"  skip (unreadable): {p}")
            continue

        if size is None:
            size = (img.shape[1], img.shape[0])
        elif (img.shape[1], img.shape[0]) != size:
            print(f"  SKIP (resolution {img.shape[1]}x{img.shape[0]} != {size[0]}x{size[1]}): {p}")
            continue

        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        ok, corners = cv2.findChessboardCorners(img, pattern, flags)
        if not ok:
            print(f"  no board found: {p}")
            continue

        corners = cv2.cornerSubPix(img, corners, refine_win, (-1, -1), criteria)
        img_points.append(corners)
        used.append(p)
        print(f"  ok: {p}")

    return img_points, used, size


def coverage_report(img_points, size, grid=3):
    """Report which cells of a 3x3 grid contain detected corners.

    Distortion coefficients are only constrained by data near the frame edges,
    so empty corner cells mean k1/k2 are extrapolated guesses.
    """
    hits = np.zeros((grid, grid), dtype=int)
    w, h = size
    for corners in img_points:
        for x, y in corners.reshape(-1, 2):
            cx = min(int(x / w * grid), grid - 1)
            cy = min(int(y / h * grid), grid - 1)
            hits[cy, cx] += 1
    return hits


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("images", help="glob for calibration images, e.g. 'captures/*.jpg'")
    ap.add_argument("--cols", type=int, default=9, help="inner corners across (default 9)")
    ap.add_argument("--rows", type=int, default=6, help="inner corners down (default 6)")
    ap.add_argument("--square", type=float, default=25.0, help="square size in mm (default 25)")
    ap.add_argument("--free-k3", action="store_true",
                    help="let k3 float; default fixes it to 0, which is more stable at low res")
    ap.add_argument("--drop-above", type=float, default=None,
                    help="re-run calibration after dropping images with per-image error above this (px)")
    ap.add_argument("--out", default="xiao_intrinsics.json")
    ap.add_argument("--undistort-preview", default=None,
                    help="write an undistorted copy of this image for a visual check")
    args = ap.parse_args()

    paths = sorted(glob.glob(args.images))
    if not paths:
        sys.exit(f"No images matched {args.images!r}")
    print(f"Found {len(paths)} images. Detecting {args.cols}x{args.rows} inner corners...")

    pattern = (args.cols, args.rows)
    img_points, used, size = find_corners(paths, pattern)

    if len(img_points) < 10:
        sys.exit(f"\nOnly {len(img_points)} usable images. Need 15+ (20-30 is better). "
                 f"Common causes: blurry frames, a non-flat board, or wrong --cols/--rows.")

    # Object points: board is the world origin, Z=0, in millimetres.
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    obj_points = [objp] * len(img_points)

    flags = 0 if args.free_k3 else cv2.CALIB_FIX_K3

    def run(op, ip):
        rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(op, ip, size, None, None, flags=flags)
        per_img = []
        for i in range(len(op)):
            proj, _ = cv2.projectPoints(op[i], rvecs[i], tvecs[i], K, dist)
            a = np.asarray(ip[i], dtype=np.float64).reshape(-1, 2)
            b = np.asarray(proj, dtype=np.float64).reshape(-1, 2)
            err = float(np.sqrt(np.mean(np.sum((a - b) ** 2, axis=1))))
            per_img.append(err)
        return rms, K, dist, per_img

    rms, K, dist, per_img = run(obj_points, img_points)

    if args.drop_above is not None:
        keep = [i for i, e in enumerate(per_img) if e <= args.drop_above]
        dropped = [used[i] for i in range(len(used)) if i not in keep]
        if dropped and len(keep) >= 10:
            print(f"\nDropping {len(dropped)} image(s) over {args.drop_above} px:")
            for d in dropped:
                print(f"  {d}")
            img_points = [img_points[i] for i in keep]
            used = [used[i] for i in keep]
            obj_points = [objp] * len(img_points)
            rms, K, dist, per_img = run(obj_points, img_points)

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    d = dist.ravel()

    print("\n" + "=" * 58)
    print(f"Images used:        {len(img_points)}")
    print(f"Resolution:         {size[0]} x {size[1]}")
    print(f"Overall RMS error:  {rms:.4f} px")
    print(f"Worst image error:  {max(per_img):.4f} px  ({used[int(np.argmax(per_img))]})")
    print("-" * 58)
    print(f"fx = {fx:10.4f}    fy = {fy:10.4f}")
    print(f"cx = {cx:10.4f}    cy = {cy:10.4f}")
    print(f"dist = [{', '.join(f'{v:.6f}' for v in d)}]   (k1 k2 p1 p2 k3)")
    print("=" * 58)

    # Sanity checks against expected values for a well-behaved fit.
    if abs(cx - size[0] / 2) > 0.15 * size[0] or abs(cy - size[1] / 2) > 0.15 * size[1]:
        print("WARN: principal point is far from image centre. Usually means poor "
              "pose variety or a non-flat target.")
    if abs(fx - fy) / max(fx, fy) > 0.05:
        print("WARN: fx and fy differ by >5%. OV2640 pixels are square, so this "
              "suggests the driver is stretching or cropping the frame.")
    if rms > 0.8:
        print("WARN: RMS above 0.8 px. Re-shoot with a rigid, flat board and sharper focus.")

    hits = coverage_report(img_points, size)
    print("\nCorner coverage (3x3 grid over the frame):")
    for row in hits:
        print("  " + "  ".join(f"{v:5d}" for v in row))
    if (hits == 0).any():
        print("  WARN: empty cells. Distortion terms are unconstrained there — "
              "shoot more views with the board pushed into the frame corners.")

    result = {
        "resolution": {"width": size[0], "height": size[1]},
        "fx": fx, "fy": fy, "cx": cx, "cy": cy,
        "dist_coeffs": d.tolist(),
        "rms_reprojection_error_px": float(rms),
        "n_images": len(img_points),
        "square_size_mm": args.square,
        "pattern_inner_corners": [args.cols, args.rows],
    }
    with open(args.out, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\nWrote {args.out}")

    if args.undistort_preview:
        img = cv2.imread(args.undistort_preview)
        if img is not None:
            newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, size, 1, size)
            out = cv2.undistort(img, K, dist, None, newK)
            name = "undistorted_preview.png"
            cv2.imwrite(name, np.hstack([img, out]))
            print(f"Wrote {name} (original | undistorted)")


if __name__ == "__main__":
    main()
