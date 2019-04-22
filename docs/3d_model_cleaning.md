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
The repository includes some handy Python scripts in the `scripts` directory. You can ignore all the C++ code.
1. Install [Open3D](http://www.open3d.org/docs/getting_started.html), `numpy`, and `matplotlib`.
2. Download this repository: `git clone https://github.com/samarth-robo/contactdb_utils`.
3. Download the data as mentioned in [Using Processed Data](#processed-data), and make the symlink: `ln -s DOWNLOAD-DIR data/contactdb_data`.
4. If you want to perform machine learning experiments, check out the [contactdb_prediction](https://github.com/samarth-robo/contactdb_prediction) repository.
### Using Raw Data
Our code is a ROS package that has been tested on Ubuntu 16.04 LTS with ROS Kinetic Kame.
1. [Install dependencies](docs/deps.md)
2. Set up a [Catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) in `~/catkin_ws`.
3. Download this repository and compile the ROS package:
```
cd ~/catkin_ws/src
git clone https://github.com/samarth-robo/contactdb_utils
cd ..
catkin_make
```
4. Download the data as mentioned in [Using Raw Data](#raw-data), and make the symlink: `ln -s DOWNLOAD-DIR data/contactdb_data`.

## Visualizing Contact Maps
This script applies our post-processing to the contact map, and shows it in interactive 3D. You need to either download the processed data, or process the raw data before running it.
```
cd contactdb_utils/scripts
python show_contactmap.py --object_name <object name> --session <participant number 1-50> --instruction <use | handoff>
```
<img src="contactmap_example.gif" style="display: block;margin-left: auto;,margin-right: auto;width: 40%"></img>

## Documents:
- [Recording your own data](docs/recording_steps.md)
- [Cleaning up 3D models of objects](docs/3d_model_processing.md)
- [Proessing the recorded data](docs/processing_steps.md)
- [Various files produced during data processing](docs/data_files.md)

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

# Remeshing
Use this for meshes that have very large faces.

## [Using Meshlab](https://compvis.quora.com/Processing-meshes-Part-1)

1. Poisson Disk sampling: 50k

2. Compute normals: 15 neighbors

3. Poisson Surface Reconstruction: octree = 10, # points per node = 5

4. Orient all faces coherently

5. (Optional) invert face orientation

## [Using Blender](https://compvis.quora.com/Processing-meshes-Part-2)
