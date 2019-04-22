# [ContactDB: Analyzing and Predicting Grasp Contact via Thermal Imaging](https://contactdb.cc.gatech.edu)
This repository contains code to create the human grasp contact maps, presented in the paper 

[ContactDB: Analyzing and Predicting Grasp Contact via Thermal Imaging](https://contactdb.cc.gatech.edu) - [Samarth Brahmbhatt](https://samarth-robo.github.io/), [Cusuh Ham](https://cusuh.github.io/), [Charles C. Kemp](http://ckemp.bme.gatech.edu/), and [James Hays](https://www.cc.gatech.edu/~hays/), CVPR 2019

[Paper (CVPR 2019 Oral)](https://arxiv.org/abs/1904.06830) | [Explore the dataset](https://contactdb.cc.gatech.edu/contactdb_explorer.html) | Poster | Slides

Please see [contactdb_prediction](https://github.com/samarth-robo/contactdb_prediction) for code to perform the contactmap prediction experiments presented in the paper.

This code is **in the process of being documented**. Feel free to open an issue if you need urgent help.

## Dataset Download:
We offer both processed and raw forms of the data.
### Processed Data
[Contact Maps (Textured Meshes) (11.5 GB)](https://www.dropbox.com/sh/gzwk21ssod63xdl/AAAJ5StPMS2eid2MnZddBGsca?dl=0). If you also need the RGBD-Thermal images, 6-DOF object poses, and image masks, use [this Dropbox link (91 GB)](https://www.dropbox.com/sh/yjp1s73ollrfafi/AAATWS-1l-MzUcNtahR36fB-a?dl=0) instead.
### Raw Data
[ROS bagfiles (1.46 TB)](https://www.dropbox.com/sh/hn90i9qglddnfpb/AABfB3pd34nkEF7_usktvVLMa?dl=0): Compressed 30 Hz RGB-D-Thermal data streams. You will also need the [Object 3D Models (180 MB)](https://www.dropbox.com/sh/jdndpjhmq9pabgi/AADRBXURc97_tPsQKCy1Zj60a?dl=0) to create contact maps from this raw data.

## Setup
### Using Processed Data
1. Install [Open3D](http://www.open3d.org/docs/getting_started.html), `numpy`, and `matplotlib`.
2. Download this repository: `git clone https://github.com/samarth-robo/contactdb_utils`.
3. If you want to perform machine learning experiments, check out the [contactdb_prediction](https://github.com/samarth-robo/contactdb_prediction) repository.
### Using Raw Data
Our code is a ROS package that has been tested on Ubuntu 16.04 LTS with ROS Kinetic Kame.
1. Download [this fork](https://github.com/samarth-robo/Open3D/tree/surface_normals_for_colormapping) of Open3D. Make sure to get the `surface_normals_for_colormapping` branch. Compile it from source ([instructions](http://www.open3d.org/docs/compilation.html)).
2. Install [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation) (the `ros-kinetic-desktop-full` version).
3. Install some dependencies: `sudo apt-get install `
4. Set up a [Catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) in `~/catkin_ws`.
5. Download this repository and compile the ROS package:
```
cd ~/catkin_ws/src
git clone https://github.com/samarth-robo/contactdb_utils
cd ..
catkin_make
```

## Visualizing Contact Maps
```
cd contactdb_utils/scripts
python show_contactmap.py --object_name <object name> --session <participant number 1-50> --instruction <use | handoff>
```
<img src="contactmap_example.gif" style="display: block;margin-left: auto;,margin-right: auto;width: 40%"></img>

## Citation
```
@inproceedings{brahmbhatt2018contactdb,
  title={{ContactDB: Analyzing and Predicting Grasp Contact via Thermal Imaging}},
  author={Samarth Brahmbhatt and Cusuh Ham and Charles C. Kemp and James Hays},
  booktitle={IEEE Conference on Computer Vision and Pattern Recognition (CVPR)},
  year={2019},
  note={\url{https://contactdb.cc.gatech.edu}}
}
```

# Dependencies

`sudo apt-get install python-qt4`

`sudo apt-get install ros-kinetic-ar-track-alvar ros-kinetic-camera-info-manager-py`

pip:
- meshrender
- numpy >= 1.15
- matplotlib

Open3D, [this branch](https://github.com/samarth-robo/Open3D/tree/surface_normals_for_colormapping).

[ip_basic](https://github.com/kujason/ip_basic) for depth map hole filling.


Need the file `~/enable_ros.sh` which contains the following:
```
#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

Check that `#include <pcl/registration/boost.h>` is included in PCL's header file `ROOT/registration/warp_point_rigid.h`.

## Installing Horus
1. Remove system opencv: `sudo apt-get remove libopencv-dev && sudo apt-get autoremove`

2. Install dependencies from [here](https://github.com/LibreScanner/horus/blob/develop/doc/development/ubuntu.md)

3. Add the `horus` PPA

4. Install `python-opencv`

5. Add the `horus-dev` PPA

6. Install `horus`

# Permissions

`sudo usermod -a -G dialout $USER` and reboot.

# Remeshing
Use this for meshes that have very large faces.

## [Using Meshlab](https://compvis.quora.com/Processing-meshes-Part-1)

1. Poisson Disk sampling: 50k

2. Compute normals: 15 neighbors

3. Poisson Surface Reconstruction: octree = 10, # points per node = 5

4. Orient all faces coherently

5. (Optional) invert face orientation

## [Using Blender](https://compvis.quora.com/Processing-meshes-Part-2)

# Various TXT files

- `object_flip.txt`: `[X Y Z RX RY RZ]`. X Y and Z in cm, used to translate the object
model before doing any pose estimation. This is mainly used for increasing the
circle radius for better circle fitting. RX RY RZ in degrees, used to rotate the
the object model before doing PE. Used for flipped objects.

- `object_name.txt`: canonical name of object, needed to look up the 3D model
from `deepgrasp_data/models`

- `recording.txt`: name of the bag file to use from the `object/bags` directory

- `scale.txt`: scale determined by the `ICP-GUI`

- `poses/tt_base.txt`: pose of the turntable base, from the bag file

- `poses/tt_base_processed.txt`: Line 1: refined pose of the turntable base. Same
translation as bag file, but rotation is from the plane estimate from pointcloud.
Line 2: plane params `[nx ny nz d]`

- All poses are in the form `X Y Z R0...R8` where XYZ are in meters, `R0-R8` are
rotation matrix values read out in row-major format

- `views.txt`: lists the names of the views that will be processed by the
automatic ICP program.

~~TODO: remove `scale.txt` and `object_scaled.ply` from all directories. Keep only
1 properly scaled PLY model in `deepgrasp_data/models` directory.~~

# Data Processing

1. Run `scripts/make_object_dirs.py`

2. Run `scripts/flip_object.py` if you want to apply a constant flip to all
objects. We typically use `dy = 5`.

3. Go through the object directories and check that the object orientation
matches the PLY model. If not, add flipping information to its `object_flip.txt`.
Imagine the PLY model's axes will be laid out on the turntable like so:
X (right), Y (forward), Z (up). For 180 degree flips, prefer to flip along
the shift axis (Y axis typically). Objects to typically look out for:
    - banana
    - camera
    - cell_phone
    - flashlight
    - flute
    - hammer
    - headphones
    - knife
    - light_bulb-n
    - ps_controller
    - utah_teapot
    
4. If you don't want a particular view to be included in a merge, delete its
`merge.txt`.

5. Very rarely, you will have to exclude some views from the ICP process. You
can do that by adding their ID (e.g. 00, 02, ...) to `excluded_views.txt`. This
is typically needed where you can't see enough information to disambiguate
the pose.
Objects to look out for:
    - camera: lens is not visible
    - cell_phone: antenna is not visible
    - mug: handle is not visible
    - light_bulb: stem is not visible
    - toothbrush: roughly parallel to camera axis
    
6. `first_view.txt`:
    - cell_phone
    - door_knob
    - flute
    - knife
    - stapler
    - toothbrush

7. Once you are done processing the session, use `scripts/delete_pointclouds.py`
to delete all the pointclouds, they occupy a lot of disk space
