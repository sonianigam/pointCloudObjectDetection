import pcl
import numpy as np
import matplotlib.pyplot as plt
from mayavi.mlab import *
from sklearn.cluster import DBSCAN

csv = open('final_project_data/final_project_point_cloud.fuse', 'rb')
point_info = []
X = []
Y = []
Z = []

#file in form of: [latitude] [longitude] [altitude] [intensity]
for line in csv:
    r = line.strip().split(' ')
    point = []
    point.append(float(r[0]))
    X.append(float(r[0]))
    point.append(float(r[1]))
    Y.append(float(r[1]))
    point.append(float(r[2]))
    Z.append(float(r[2]))
    point_info.append(point)

##this is the passage we need to follow for filtering
'''
Our first step is to remove points that clearly are not part
of small objects. We filter out points close to the ground,
which is estimated at uniformly spaced positions with iterative
plane fitting. We then remove isolated points. Finally,
we filter out points likely to belong to buildings by removing
very large connected components. Once these filters
have been run, we proceed with one of four approaches to
find potential object locations.

Since objects of interest are likely to rise above their local
surroundings, a simple approach to finding potential object
locations is to generate a 2D scalar image representing
the "height" of the point cloud, and performing image processing
operations to find local maxima. We experimented
with several variations of this approach. The most successful
variant is to generate an image using the maximum
height of the points in each pixel, run a difference of Gaussian
filters to find high frequencies, and then extract connected
components with area under a threshold to find small
objects. This method is effective at finding isolated poles,
but performs worse for cars and objects amongst clutter.
'''

#make point cloud from fuse file data
point_info = np.array(point_info, dtype=np.float32)
p = pcl.PointCloud()
p.from_array(point_info)
print p

##remove points close to the ground (not sure this is necessary) -- revisit at the end

#removes outlier data based on KNN
filter1 = p.make_statistical_outlier_filter()
filter1.set_mean_k(50)
filter1.set_std_dev_mul_thresh(5.0)

filter1.set_negative(False)
filtered_cloud = filter1.filter()

print filtered_cloud 

##need to generate 2d scalar image

#######################################
##segment the chosen objects --> cylinders
seg = filtered_cloud.make_segmenter()
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC )
seg.set_distance_threshold(100.0)
segmented_indices, model = seg.segment()
# print segmented_indices

#return just cylinder segments
filtered_cloud = filtered_cloud.extract(segmented_indices, negative=False)
print filtered_cloud
##################################

##need to remove largely connected components --> likely mountains in background/road
kd = filtered_cloud.make_kdtree_flann()
indices, sqr_distances = kd.nearest_k_search_for_cloud(filtered_cloud, 1000)

distances = np.sum(sqr_distances, axis=1)

remove_indices = []

for i in xrange(np.shape(distances)[0]):
    if distances[i] < 1.0:
        remove_indices.extend(indices[i])

remove_unique_indices = list(set(remove_indices))
print len(remove_unique_indices)
filtered_cloud = filtered_cloud.extract(remove_unique_indices, negative=True)
print filtered_cloud

##need to generate 2d scalar image

##segment the chosen objects --> cylinders
seg = filtered_cloud.make_segmenter()
# seg.set_model_type(pcl.SACMODEL_CYLINDER)
# seg.set_method_type(pcl.SAC_LMEDS )
# segmented_indices, model = seg.segment()
# print segmented_indices

#return just cylinder segments
# filtered_cloud = filtered_cloud.extract(segmented_indices, negative=False)
print filtered_cloud


new_X = []
new_Y = []
new_Z = []
for x in filtered_cloud:
    new_X.append(x[0])
    new_Y.append(x[1])
    new_Z.append(x[2])

plt.tripcolor(new_X,new_Y,new_Z, cmap = 'Greens')
plt.plot(new_X,new_Y, '.')
plt.show()