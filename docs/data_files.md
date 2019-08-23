# [ContactDB: Analyzing and Predicting Grasp Contact via Thermal Imaging](https://contactdb.cc.gatech.edu)
Explanation for various data files in each session directory e.g. `full10_use/banana`.

- `depth_images`, `rgb_images`, `thermal_images`: Directories containing synchronized images from 3 modalities.
Views are named `00`, `01`, and so on.

- `rgb_images/xx.png` is the original RGB image from the Kinect (960 x 540), while `rgb_images/xx_registered.png` is registered to the thermal camera. The registered image might be blocky because of imperfections in the depth map hole filling algorithm and because of the change in viewpoint.

- `depth_images/xx.png` is the raw depth image registered to the thermal camera, while `depth_images/xx_registered.png` is the post-processed version of this depth image, smoothed out and hole filled. Not the confusing naming pattern, both images are actually registered to the thermal camera. 

- `poses`: Directory containing all the object/camera pose information.

- `poses/tt_base.txt`: pose of the turntable base, from the bag file

- `poses/tt_base_processed.txt`: Line 1: refined pose of the turntable base. Same
translation as bag file, but rotation is from the plane estimate from pointcloud.
Line 2: plane params `[nx ny nz d]`

- `poses/tt_frame_xx.txt`: Line 1 has the pose of turntable frame w.r.t. its base (i.e. the turntable angle),
Line 2 has the pose of the turntable frame w.r.t. Kinect.

- These poses are in the form `X Y Z R0...R8` where XYZ are in meters, `R0-R8` are
rotation matrix values read out in row-major format.

- `poses/camera_pose_xx.txt`: pose of Kinect w.r.t. object for view `xx`, used by the `check_camera_poses`
executable in the ROS package.

- `poses/object_pose_xx.txt`: pose of object w.r.t. thermal camera, used by `scripts/texture_mapping.py`.

- `object_flip.txt`: `[X Y Z RX RY RZ]`. X Y and Z in cm, used to translate the object
model before doing any pose estimation. This is mainly used for increasing the
circle radius for better circle fitting. RX RY RZ in degrees, used to rotate the
the object model before doing PE. Used for flipped objects.

- `object_name.txt`: canonical name of object, needed to look up the 3D model
from `data/contactdb_3d_models`

- `recording.txt`: name of the ROS bag file for this session

- `scale.txt`: scale to be applied to the object mesh before texture mapping. Set to `1` for now.

- `excluded_views.txt`: If non-empty, lists the names of the views that will be ignored by the
automatic ICP program.

- `first_view.txt`: If non-empty, specifies the name of the view for which
`object_flip.txt` was written.

- `merge.txt`: When multiple scans of the object are used to create one contact map, they are stored
in directories like `apple`, `apple-1`, `apple-2`, and so on. Each of the scans to be included in the
final contact map has a `merge.txt` in its directory. The main scan where all the information is 
gathered has a `1` in its `merge.txt`, all other `merge.txt`s are empty.
