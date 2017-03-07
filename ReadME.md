# Pole Detection
EECS 395 Final Assignment

##Run 
The following is how to filter a point cloud image for poles
How to Run

    python poleDetection.py


##Output
Pole Detection will output a file called "final.obj" in the directory you are working in with the obj file of the point cloud image with the filtered image. You can open this file in meshlab. 

In Terminal, the program will print the following:<br />
	Filtered without outliers <br />
	<PointCloud of 428148 points> <br /> 
	filtered cloud without large components <br />
	<PointCloud of 42135 points> <br />
	filtered cloud after segmentation <br />
	<PointCloud of 28605 points> <br />
	filtered cloud without plane 2.0 <br />
	<PointCloud of 27697 points> <br />
	filtered cloud without plane <br />
	<PointCloud of 25341 points> <br />

##Dependencies
Straw lab Python API for PCL: conda install -c ccordoba12 python-pcl=0.2 <br />

##Resources
http://www.cs.princeton.edu/~funk/iccv09.pdf

