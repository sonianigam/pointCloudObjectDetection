import pcl
import numpy as np
import math

csv = open('final_project_data/final_project_point_cloud.fuse', 'rb')
point_info = []
point_cloud_display = []


def cartesian(lat,lon, elevation):
    cosLat = math.cos(lat * math.pi / 180.0)
    sinLat = math.sin(lat * math.pi / 180.0)
    cosLon = math.cos(lon * math.pi / 180.0)
    sinLon = math.sin(lon * math.pi / 180.0)
    rad = 6378137.0 + elevation
    f = 1.0 / 298.257224
    C = 1.0 / math.sqrt(cosLat * cosLat + (1 - f) * (1 - f) * sinLat * sinLat)
    S = (1.0 - f) * (1.0 - f) * C
    h = 0.0
    x = (rad * C + h) * cosLat * cosLon
    y = (rad * C + h) * cosLat * sinLon
    z = (rad * S + h) * sinLat
    return x, y, z




#file in form of: [latitude] [longitude] [altitude] [intensity]
for line in csv:
    r = line.strip().split(' ')
    point = []
    x, y, z = cartesian(float(r[0]), float(r[1]), float(r[2]))
    point.append(x)
    point.append(y)
    point.append(z)
    point_info.append(point)

initial = open('pointcloud.obj', 'w')

# line1 = "mtllib ./vp.mtl"
# initial.write(line1)
# initial.write("\n")

for point in point_info:
    line = "v " + str(point[0]) + " " + str(point[1]) + " "+ str(point[2])
    initial.write(line)
    initial.write("\n")

initial.close()



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

##remove points close to the ground (not sure this is necessary) -- revisit at the end

#removes outlier data based on KNN
filter1 = p.make_statistical_outlier_filter()
filter1.set_mean_k(50)
filter1.set_std_dev_mul_thresh(5.0)

filter1.set_negative(False)
filtered_cloud = filter1.filter()

print "Filtered without outliers"
print filtered_cloud 

##need to generate 2d scalar image

##need to remove largely connected components --> likely mountains in background/road
kd = filtered_cloud.make_kdtree_flann()
indices, sqr_distances = kd.nearest_k_search_for_cloud(filtered_cloud, 1000)

distances = np.sum(sqr_distances, axis=1)
remove_indices = []
print "distance max and min"
print max(distances)
print min(distances)
for i in xrange(np.shape(distances)[0]):
    if distances[i] < 10000.0:
        remove_indices.extend(indices[i])

remove_unique_indices = list(set(remove_indices))
"the len of large components to be removed"
print len(remove_unique_indices)
filtered_cloud = filtered_cloud.extract(remove_unique_indices, negative=True)
print "filtered cloud without large components"
print filtered_cloud

##need to generate 2d scalar image

#######################################
##segment the chosen objects --> cylinders
seg = filtered_cloud.make_segmenter_normals(ksearch=50)
seg.set_optimize_coefficients(True)
seg.set_model_type(pcl.SACMODEL_CYLINDER)
seg.set_normal_distance_weight(0.1)
seg.set_method_type(pcl.SAC_RANSAC)
seg.set_max_iterations(1000)
seg.set_distance_threshold(30)  #the lower the number is, the more stuff it will take out
seg.set_radius_limits(0, 0.1) #what do you think the radius of the pole is?

segmented_indices, model = seg.segment()
# print segmented_indices

#return just cylinder segments - FEATURE EXTRACTION
filtered_cloud = filtered_cloud.extract(segmented_indices, negative=False)
print "filtered cloud after segmentation"
print filtered_cloud
##################################

final = open('final.obj', 'w')

# line1 = "mtllib ./vp.mtl"
# initial.write(line1)
# initial.write("\n")

for point in filtered_cloud:
    line = "v " + str(point[0]) + " " + str(point[1]) + " "+ str(point[2])
    final.write(line)
    final.write("\n")

