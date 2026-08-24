#!/usr/bin/env python3
"""
Live pose readout for the XIAO ESP32S3 Sense.

Same maths as check_pose.py, but continuous and with a preview window, so you
can position the target while watching the numbers instead of firing a single
blind frame.

    py live_pose.py --intrinsics xiao_intrinsics.json --mode board \
        --square 18.3 --url http://192.168.1.235/capture --expect 300

    py live_pose.py --intrinsics xiao_intrinsics.json --mode tag \
        --tag-size 52 --url http://192.168.1.235/capture --expect 500

Keys:  s = print a line to the terminal    q = quit

THE TWO NUMBERS THAT MATTER FOR DIAGNOSIS
-----------------------------------------
tilt        pitch/yaw/roll of the target relative to the camera. solvePnP
            returns the pose of the board's ORIGIN CORNER, not its centre.
            On a tilted board those differ by up to half the board diagonal,
            so a tape measurement to the centre will disagree with Z for
            purely geometric reasons. Get tilt under ~5 degrees before
            trusting any comparison against a tape.

implied     what --square (or --tag-size) would have to be for Z to equal
            --expect. If this parks on the value you actually measured with
            calipers, your scale input is right and any earlier disagreement
            came from tilt or from measuring to the wrong reference point.
            If it parks somewhere else and holds steady across distances,
            your scale input is genuinely wrong.

A scale error MULTIPLIES with distance. A reference-point error ADDS a
constant. Take readings at two distances to tell them apart.
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


def grab(session, url):
    r = session.get(url, timeout=5)
    r.raise_for_status()
    arr = np.frombuffer(r.content, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


def euler_deg(rvec):
    """Rotation vector -> (pitch, yaw, roll) in degrees, for a rough
    'is this thing square-on' readout. Not a rigorous convention, just a
    tilt indicator."""
    R, _ = cv2.Rodrigues(rvec)
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0.0
    return np.degrees([x, y, z])


def put(img, text, y, color=(255, 255, 255), scale=0.5):
    cv2.putText(img, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3)
    cv2.putText(img, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--intrinsics", default="xiao_intrinsics.json")
    ap.add_argument("--url", required=True)
    ap.add_argument("--mode", choices=["board", "tag"], required=True)
    ap.add_argument("--expect", type=float, default=None,
                    help="tape-measured distance in mm")
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square", type=float, default=18.3, help="mm")
    ap.add_argument("--tag-size", type=float, default=52.0,
                    help="mm, OUTER EDGE OF THE BLACK SQUARE")
    ap.add_argument("--scale", type=float, default=1.5, help="preview magnification")
    args = ap.parse_args()

    K, dist, calib_size, raw = load_intrinsics(args.intrinsics)
    print(f"Intrinsics: {calib_size[0]}x{calib_size[1]}  fx={raw['fx']:.2f} "
          f"fy={raw['fy']:.2f}  RMS {raw['rms_reprojection_error_px']:.4f}")
    print("s = print reading    q = quit\n")

    detector = None
    if args.mode == "tag":
        try:
            from pupil_apriltags import Detector
        except ImportError:
            sys.exit("py -m pip install pupil-apriltags")
        detector = Detector(families="tag36h11", nthreads=2,
                            quad_decimate=1.0, decode_sharpening=0.25)
        newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, calib_size, 0, calib_size)
        cam_params = (newK[0, 0], newK[1, 1], newK[0, 2], newK[1, 2])
    else:
        objp = np.zeros((args.rows * args.cols, 3), np.float32)
        objp[:, :2] = (np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
                       * args.square)
        newK = None

    pattern = (args.cols, args.rows)
    cb_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    session = requests.Session()
    reading = None

    while True:
        try:
            img = grab(session, args.url)
        except Exception as e:
            print(f"  fetch failed: {e}")
            continue
        if img is None:
            continue

        h, w = img.shape[:2]
        if (w, h) != calib_size:
            sys.exit(f"Frame is {w}x{h}, intrinsics are for "
                     f"{calib_size[0]}x{calib_size[1]}. These do not apply.")

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        vis = img.copy()
        z_mm = None
        tilt = None
        label = "no target"

        if args.mode == "board":
            ok, corners = cv2.findChessboardCorners(gray, pattern, cb_flags)
            if ok:
                corners = cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1),
                                           criteria)
                cv2.drawChessboardCorners(vis, pattern, corners, ok)
                ok2, rvec, tvec = cv2.solvePnP(objp, corners, K, dist)
                if ok2:
                    cv2.drawFrameAxes(vis, K, dist, rvec, tvec,
                                      args.square * 3)
                    z_mm = float(tvec.ravel()[2])
                    tilt = euler_deg(rvec)
                    label = "board"
        else:
            und = cv2.undistort(gray, K, dist, None, newK)
            gray = und
            vis = cv2.cvtColor(und, cv2.COLOR_GRAY2BGR)
            res = detector.detect(und, estimate_tag_pose=True,
                                  camera_params=cam_params,
                                  tag_size=args.tag_size / 1000.0)
            if res:
                r = res[0]
                c = r.corners.astype(int)
                cv2.polylines(vis, [c], True, (0, 220, 0), 2)
                px = float(max(np.linalg.norm(r.corners[i] - r.corners[(i + 1) % 4])
                               for i in range(4)))
                z_mm = float(r.pose_t.ravel()[2] * 1000.0)
                tilt = euler_deg(cv2.Rodrigues(np.array(r.pose_R))[0])
                label = f"tag ID {r.tag_id}   {px:.0f} px wide"
                if px < 25:
                    label += "  (UNDER 25 px - unreliable)"

        if args.scale != 1.0:
            vis = cv2.resize(vis, None, fx=args.scale, fy=args.scale,
                             interpolation=cv2.INTER_NEAREST)

        y = 22
        put(vis, label, y, (0, 220, 0) if z_mm is not None else (0, 0, 255))
        if z_mm is not None:
            y += 22
            put(vis, f"Z = {z_mm:7.1f} mm", y, (0, 220, 255), 0.62)

            worst = float(np.max(np.abs(tilt[:2])))
            tilt_col = (0, 0, 255) if worst > 10.0 else (0, 220, 0)
            y += 22
            put(vis, f"tilt  p{tilt[0]:+6.1f}  y{tilt[1]:+6.1f}  r{tilt[2]:+6.1f} deg",
                y, tilt_col)
            if worst > 10.0:
                y += 20
                put(vis, "TILTED - Z is to the origin corner, not the centre",
                    y, (0, 0, 255), 0.45)

            if args.expect:
                stated = args.square if args.mode == "board" else args.tag_size
                implied = stated * args.expect / z_mm
                pct = 100.0 * (z_mm - args.expect) / args.expect
                col = (0, 220, 0) if abs(pct) < 2.0 else (0, 165, 255)
                y += 22
                put(vis, f"vs expect {args.expect:.0f}: {pct:+.1f}%", y, col)
                y += 20
                put(vis, f"implied size = {implied:.2f} mm  (you said {stated:.2f})",
                    y, col)
                reading = (z_mm, pct, implied, tilt)

        cv2.imshow("live pose", vis)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s") and reading:
            z, pct, implied, t = reading
            print(f"Z={z:7.1f} mm   {pct:+6.1f}%   implied={implied:6.2f} mm   "
                  f"tilt p{t[0]:+.1f} y{t[1]:+.1f} r{t[2]:+.1f}")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
