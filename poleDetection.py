import pcl
import numpy as np
import matplotlib as plt

csv = open('final_project_data/final_project_point_cloud.fuse', 'rb')
point_info = []

#file in form of: [latitude] [longitude] [altitude] [intensity]
for line in csv:
    r = line.strip().split(' ')
    point = []
    point.append(float(r[0]))
    point.append(float(r[1]))
    point.append(float(r[2]))

    point_info.append(point)
point_info = np.array(point_info, dtype=np.float32)

p = pcl.PointCloud()
p.from_array(point_info)

