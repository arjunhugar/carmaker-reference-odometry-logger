# CarMaker Reference Odometry + Object Sensor Logger (C)

This repository contains a minimal example of extending a CarMaker user module to log:
- vehicle odometry (position, velocity, acceleration, yaw rate, gear)
- object sensor outputs (target IDs, positions, velocities, dimensions)

## What it generates
During a test run, the code writes two files to `SimOutput/reference_odometry_output/`:
- `Car_output_<timestamp>.txt`
- `Object_Sensor_output_<timestamp>.txt`

## Output directory
By default, output is written to a local folder:
- `SimOutput/`

You can override output directory via environment variable:
- `CM_OUTDIR=<your_path>`

## Build notes
This source requires a valid CarMaker SDK/project environment to compile because it includes CarMaker headers.

This repository is intended for portfolio/educational demonstration of logging and data export patterns.

## Logged signals (examples)
### Car output
- time stamp
- position (x,y,z)
- velocity (x,y,z)
- acceleration (x,y,z) with clipping
- yaw rate, gear

### Object sensor output
- time stamp
- object ID and traffic ID (if available)
- reference point position / velocity / orientation
- dimensions (height, length, width)
