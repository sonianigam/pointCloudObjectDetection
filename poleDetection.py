import pcl
import numpy as np
import math

csv = open('final_project_data/final_project_point_cloud.fuse', 'rb')
point_info = []
point_cloud_display = []

#converts lat, lon, elevation into xyz coordinates 
#source: http://stackoverflow.com/questions/28365948/javascript-latitude-longitude-to-xyz-on-sphere-three-js
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
#write original point cloud data into a file
for line in csv:
    r = line.strip().split(' ')
    point = []
    x, y, z = cartesian(float(r[0]), float(r[1]), float(r[2]))
    point.append(x)
    point.append(y)
    point.append(z)
    point_info.append(point)

initial = open('pointcloud.obj', 'w')

#write file in necessary format for .obj
for point in point_info:
    line = "v " + str(point[0]) + " " + str(point[1]) + " "+ str(point[2])
    initial.write(line)
    initial.write("\n")

initial.close()

#make point cloud from fuse file data
point_info = np.array(point_info, dtype=np.float32)
p = pcl.PointCloud()
p.from_array(point_info)

print "Original point cloud"
print p

#removes outlier data based on 50 KNN and 5 SD threshold
filter1 = p.make_statistical_outlier_filter()
filter1.set_mean_k(50)
filter1.set_std_dev_mul_thresh(5.0)

#return all points that are not outliers
filter1.set_negative(False)
filtered_cloud = filter1.filter()

print "Filtered without outliers"
print filtered_cloud 


##remove largely connected components -->  mountains in background/road
kd = filtered_cloud.make_kdtree_flann()
indices, sqr_distances = kd.nearest_k_search_for_cloud(filtered_cloud, 1000)

#sum distances per derived component
distances = np.sum(sqr_distances, axis=1)
remove_indices = []

#if summed distance less than 5000, dense enough to be removed
for i in xrange(np.shape(distances)[0]):
    if distances[i] < 5000.0:
        remove_indices.extend(indices[i])

#remove repetitive indices
remove_unique_indices = list(set(remove_indices))

#return indices not in large components
filtered_cloud = filtered_cloud.extract(remove_unique_indices, negative=True)
print "Filtered  without large components"
print filtered_cloud


##clyindrical segmentation: discard all indices not found within isolated cylindrical segments
seg = filtered_cloud.make_segmenter_normals(ksearch=50)
seg.set_optimize_coefficients(True)
seg.set_model_type(pcl.SACMODEL_CYLINDER)
seg.set_normal_distance_weight(0.1)
seg.set_method_type(pcl.SAC_RANSAC)
seg.set_max_iterations(1000)
seg.set_distance_threshold(20) 
seg.set_radius_limits(0, 10) #the radius of the pole
segmented_indices, model = seg.segment()


#return just cylindrical segments - FEATURE EXTRACTION
filtered_cloud = filtered_cloud.extract(segmented_indices, negative=False)
print "Cloud after cylindrical segmentation"
print filtered_cloud


#segment ground plane, and then discard all indices associated with plane
seg = filtered_cloud.make_segmenter_normals(ksearch=50)
seg.set_optimize_coefficients(True)
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
seg.set_normal_distance_weight(0.1)
seg.set_distance_threshold(85) 
seg.set_max_iterations(100)
indices, model = seg.segment()

#return indices not in identified ground plane
filtered_cloud = filtered_cloud.extract(indices, negative=True)

#filter cloud data based on height
fil = filtered_cloud.make_passthrough_filter()
fil.set_filter_field_name("x")
fil.set_filter_limits(0, 4364071.0)

#apply height-based filter and return those not in bounds
filtered_cloud = fil.filter()
print "Final point cloud"
print filtered_cloud


#write final data file
final = open('final.obj', 'w')
for point in filtered_cloud:
    line = "v " + str(point[0]) + " " + str(point[1]) + " "+ str(point[2])
    final.write(line)
    final.write("\n")

