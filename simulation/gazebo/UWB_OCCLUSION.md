PeerHeightSystem optionally loads `terrain_triangles_file` during Configure.
Use an absolute path accessible to the Gazebo process (Linux path for WSL).
Format: existing companion TerrainMap raw little-endian float32 Nx3x3
triangles in world ENU metres, with the same placement/scale as the scene.
An empty path preserves existing observations. A supplied file that cannot
be loaded prevents publishing and logs an error.

The shared segment query uses the nearest static terrain mesh intersection
and excludes 0.1 mm at both endpoints. Dynamic obstacles are not included.
TerrainMap retains its existing traversal cap and lack of an indeterminate
status: a failed ray query is treated as no intersection.

This **uncalibrated stress model** uses the following parameters:

- `nlos_drop_probability`: default 0.75, additional dropout when occluded.
- `nlos_range_bias_m`: default 0.4 m, nonnegative excess range.
- `nlos_range_noise_std_m`: default 0.03 m, half-normal excess noise scale.

Baseline noise/FOV/range/dropout/ambiguity still apply, with ranges clipped
to configured sensor limits. UDP retains range/bearing/valid and adds `nlos`,
a simulated sensor quality boolean currently ignored by the companion.
No intersection point, true range or world position is published. Scalar
height assistance is unchanged.

CTest `uwb_occlusion` tests the same helper used by the plugin, with assertions
enabled in RelWithDebInfo: blockers, reciprocal paths, open paths, no mesh,
zero-length segments, beyond-peer geometry and both endpoint exclusions.
