# [ContactDB: Analyzing and Predicting Grasp Contact via Thermal Imaging](https://contactdb.cc.gatech.edu)
This repository contains code to create the human grasp contact maps, presented in the paper 

[ContactDB: Analyzing and Predicting Grasp Contact via Thermal Imaging](https://contactdb.cc.gatech.edu/contactdb_paper.pdf) - [Samarth Brahmbhatt](https://samarth-robo.github.io/), [Cusuh Ham](https://cusuh.github.io/), [Charles C. Kemp](http://ckemp.bme.gatech.edu/), and [James Hays](https://www.cc.gatech.edu/~hays/), CVPR 2019

[Paper (CVPR 2019 Oral)](https://contactdb.cc.gatech.edu/contactdb_paper.pdf) | [Supplementary Material](https://contactdb.cc.gatech.edu/contactdb_supp.pdf) | [Explore the dataset](https://contactdb.cc.gatech.edu/contactdb_explorer.html) | Poster | Slides

Please see [contactdb_prediction](https://github.com/samarth-robo/contactdb_prediction) for code to perform the contactmap prediction experiments presented in the paper.

## Setup
Our code has been tested on Ubuntu 16.04 LTS with ROS Kinetic Kame.
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

## Dataset Download links:
- [Contact Maps (Textured Meshes) (11.5 GB)](https://www.dropbox.com/sh/gzwk21ssod63xdl/AAAJ5StPMS2eid2MnZddBGsca?dl=0)
- [Data (91 GB)](https://www.dropbox.com/sh/yjp1s73ollrfafi/AAATWS-1l-MzUcNtahR36fB-a?dl=0): RGB-D-Thermal images, object 6-DOF poses and image masks, textured meshes.
- [3D Models (180 MB)](https://www.dropbox.com/sh/jdndpjhmq9pabgi/AADRBXURc97_tPsQKCy1Zj60a?dl=0)
- [Raw ROS bagfiles (1.46 TB)](https://www.dropbox.com/sh/hn90i9qglddnfpb/AABfB3pd34nkEF7_usktvVLMa?dl=0): Compressed 30 Hz RGB-D-Thermal data streams. See [this file](docs/rosbags.md) for documentation on how to process them.

## Visualizing Contact Maps

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
