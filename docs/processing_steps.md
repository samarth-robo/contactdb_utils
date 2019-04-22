# [ContactDB: Analyzing and Predicting Grasp Contact via Thermal Imaging](https://contactdb.cc.gatech.edu)
A step-by-step guide to processing your own data to create contact maps.

1. Run `scripts/make_object_dirs.py`

2. Run `scripts/flip_object.py` if you want to apply a constant flip to all
objects. We typically use `dy = 5`.

3. Go through the object directories and check that the object orientation
matches the PLY model. If not, add flipping information to its `object_flip.txt`.
Imagine the PLY model's axes will be laid out on the turntable like so:
X (right), Y (forward), Z (up). For 180 degree flips, prefer to flip along
the shift axis (Y axis typically).

4. If you don't want a particular view to be included in a merge, delete its
`merge.txt`.

5. Sometimes, you will have to exclude some views from the ICP process. You
can do that by adding their ID (e.g. 00, 02, ...) to `excluded_views.txt`. This
is typically needed where you can't see enough information to disambiguate
the pose.

6. `first_view.txt`: Specify the view corresponding to the information in
`object_flip.txt`. Necessary for aligning multiple scans of rotationally symmetric
objects, and to disambiguate the pose of 180 degree 'flippable' objects like
cell phone. A random first view is chosen if this file is empty.

7. Once you are done processing the session, use `scripts/delete_pointclouds.py`
to delete all the pointclouds, they occupy a lot of disk space
