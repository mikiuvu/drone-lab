# Companion sensor configuration

The generated model uses the Gazebo x500 flight model or the standalone
generated_micro reference model. The micro model is physical and dynamic,
but its motor and airframe parameters are reference values and are not tuned
to DM-FC01.

## Generate models

Default structure sensing uses a 20 degree downward ToF pitch:

~~~text
python3 simulation/models/generate_sensor_models.py --dynamics micro
~~~

For takeoff bootstrap, generate a separate horizontal ToF variant:

~~~text
python3 simulation/models/generate_sensor_models.py \
  --dynamics micro \
  --output simulation/models/generated_micro_horizontal \
  --tof-pitch-down-deg 0
~~~

The output directory is part of the model identity. Do not overwrite a
running 20 degree model with the horizontal variant.

## Frames and pixels

World coordinates use ENU. Vehicle and sensor coordinates use FLU: +X is
forward, +Y is left, and +Z is up. ToF pitch is a positive rotation about +Y,
so 20 means the optical axis points downward. The three ToF yaw angles are
0, +120, and -120 degrees. The ToF origin is outside the micro body at:

~~~text
sensor 0: ( 0.055000,  0.000000, -0.017000) m
sensor 1: (-0.027500,  0.047631, -0.017000) m
sensor 2: (-0.027500, -0.047631, -0.017000) m
~~~

Each ToF stream is an 8x8 axial depth image with 45 degree horizontal and
vertical FOV. Indexing is row major. For row r and column c, the unit ray
in the sensor frame is proportional to:

~~~text
x = 1
y = (1 - 2*(c + 0.5)/8) * tan(22.5 degrees)
z = (1 - 2*(r + 0.5)/8) * tan(22.5 degrees)
~~~

The returned distance is the axial X distance, not the Euclidean ray length.
The RGB camera uses +X as its forward view. Convert FLU body coordinates to
the usual optical convention with:

~~~text
(x_optical, y_optical, z_optical) = (-y_body, -z_body, x_body)
~~~

## Runtime streams

Run the gateway with the repository runtime directory:

~~~text
/home/devcontainers/drone-sim/build-sensors/sensor_gateway \
  --runtime-dir "/mnt/d/Robomaster code/drone/simulation/runtime"
~~~

UDP observation ports are 16000 + vehicle. The gateway writes
simulation/runtime/camera_i.ppm and camera_i.json using atomic replacement.
The height_assist record is a scalar ENU world Z assistance value
with source SIM_HEIGHT_ASSIST; it does not contain x, y, yaw, or velocity.
