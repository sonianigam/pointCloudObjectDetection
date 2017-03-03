import pcl
import numpy as np
import matplotlib.pyplot as plt

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
#
# plt.tripcolor(X,Y,Z)
# plt.plot(X,Y, '.')
# plt.savefig('original.png')

point_info.append(point)
point_info = np.array(point_info, dtype=np.float32)

p = pcl.PointCloud()
p.from_array(point_info)
print p
fil = p.make_statistical_outlier_filter()
fil.set_mean_k(50)
fil.set_std_dev_mul_thresh(1.0)

fil.set_negative(True)

print fil.filter()
new_X = []
new_Y = []
new_Z = []
for x in fil.filter():
    new_X.append(x[0])
    new_Y.append(x[1])
    new_Z.append(x[2])

plt.tripcolor(new_X,new_Y,new_Z)
plt.plot(new_X,new_Y, '.')
plt.show()