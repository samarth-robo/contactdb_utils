# [ContactDB: Analyzing and Predicting Grasp Contact via Thermal Imaging](https://contactdb.cc.gatech.edu)
Tricks for cleaning up 3D models and making them suitable for texture mapping. The main issue is to ensure that
the model has many small and roughly equally sized faces, rather than a few large ones. We've already done that
for the models included in ContactDB.

## [Using Meshlab](https://compvis.quora.com/Processing-meshes-Part-1)

1. Poisson Disk sampling: 50k

2. Compute normals: 15 neighbors

3. Poisson Surface Reconstruction: octree = 10, # points per node = 5

4. Orient all faces coherently

5. (Optional) invert face orientation

## [Using Blender](https://compvis.quora.com/Processing-meshes-Part-2)
