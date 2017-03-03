import pcl
import numpy as np
import math


p = pcl.PointCloud()

p.from_array(np.array([[1,2,3],[3,4,5]], dtype=np.float32))
seg = p.make_segmenter()
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
indices, model = seg.segment()