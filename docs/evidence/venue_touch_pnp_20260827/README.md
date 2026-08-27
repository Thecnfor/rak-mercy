# 2026-08-27 venue touch PnP audit overlays

This directory contains only the six final annotated overlays used to audit the
offline venue-touch PnP candidate. It does not contain the original capture set,
depth frames, or any runtime cache.

The green cross and red circle identify the same physical feature in every
frame: the outer corner of the camera-visible upper gripper jaw. The diagonal
silver/black object next to it is the pen and was not annotated. Coordinates are
in full-resolution `640 x 360` image pixels and are listed with their source
image SHA256 and measured `right_arm_sdk` Cartesian pose in [manifest.json](manifest.json).

The input was treated as `left_rectified_640x360` and solved with the venue P1
and zero distortion. Capture metadata did not preserve the source topic name;
visual review found the same rectified-left geometry/FOV, but this remains an
evidence limitation. This calibration must never be described as `base_link`
hand-eye and must never be published as TF.

## Result

- Selected candidate: `ITERATIVE` (all IPPE and iterative candidates remain in
  `Deyes/config/camera/venue_20260827_touch_projector.yaml`).
- Reprojection RMS: `4.1917268 px` — **failed** the `<= 4 px` gate.
- Reprojection P95: `5.3677005 px` — passed the `<= 6 px` gate.
- Leave-one-out Base XY RMS: `14.6408758 mm` — passed the `<= 15 mm` gate.
- Leave-one-out Base XY P95: `22.4515165 mm` — passed the `<= 25 mm` gate.
- All six points have positive camera depth; minimum depth is `0.3075441 m`.
- Final status: **`usable: false`**. The failed RMS gate prohibits pixel
  projection; retain an explicit fixed `right_arm_sdk` XY fallback.

The PNG overlays are lossless and total about 1.4 MiB, preserving exact marker
placement for review.
