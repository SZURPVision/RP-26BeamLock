# Lidar Guide Calibration - Test Data Generator

## Usage

Generate synthetic `points.txt` and `angles.txt` plus ground-truth transform:

```bash
./build/Tools/LidarGuideCalibration/lidar_guide_calibration_gen [output_dir] [count] [seed] [angle_noise_std_deg] [point_noise_std]
```

- `output_dir`: output folder (default: `CalibrationData`)
- `count`: number of samples (default: `30`, must be >= 4)
- `seed`: RNG seed (default: `42`)
- `angle_noise_std_deg`: angle noise std in degrees (default: `0`)
- `point_noise_std`: point noise std in meters (default: `0`)

Outputs:
- `points.txt`: Lidar points in lidar frame
- `angles.txt`: `pitch yaw` in degrees (clockwise convention used in solver)
- `GuideTransform_gt.yaml`: ground-truth transform (object->image frame)

## Quick Try

```bash
./build/Tools/LidarGuideCalibration/lidar_guide_calibration_gen CalibrationData 40 7
./build/Tools/LidarGuideCalibration/lidar_guide_calibration_once CalibrationData
```

## Encoder Monitor

Reset the gimbal, disable motor torque (`control=false`), then stream encoder angles and relative angles.

```bash
./build/Tools/LidarGuideCalibration/lidar_guide_encoder_monitor
```

## Transform Reader (YAML)

Read calibration result from `GuideTransform.yaml` (cv::FileStorage format). Simple prompt loop.

```bash
./build/Tools/LidarGuideCalibration/lidar_guide_point_to_angle [GuideTransform.yaml] [--send]
```

- `--send`: also forward calculated gimbal angles to the controller.

## Transform Reader (Rotation Matrix TXT)

Read calibration result from a plain-text file. Monitors file timestamp — when the file is modified externally it auto-reloads the transform. User input runs in a background thread.

**File format** (`GuideTransform.txt`):

```
tx ty tz
<empty line>
r00 r01 r02
r10 r11 r12
r20 r21 r22
```

```bash
./build/Tools/LidarGuideCalibration/lidar_guide_point_to_angle_rotation [GuideTransform.txt] [--send]
```

## Transform Reader (Euler Angles TXT)

Same as above, but reads ZYX intrinsic Euler angles (yaw / pitch / roll in degrees) instead of a full rotation matrix.

**File format** (`GuideTransform.txt`):

```
tx ty tz
<empty line>
yaw pitch roll
```

```bash
./build/Tools/LidarGuideCalibration/lidar_guide_point_to_angle_eular [GuideTransform.txt] [--send]
```
