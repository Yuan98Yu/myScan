# 参考https://www.cnblogs.com/AllStarGIS/p/5143562.html

from scipy.spatial import Delaunay
import numpy as np
import matplotlib.pyplot as plt

# width = 80
# height = 40
# radius = 10
# center_point = [radius, radius]
# points = []
# for i in range(2*radius+1):
#     for j in range(2*radius+1):
#         if (i-center_point[0])**2+(j-center_point[1])**2 < radius**2:
#             points.append([i, j])
points = []
with open("download.txt") as fr:
    for line in fr.readlines():
        lineArr = line.strip().split(',')
        points.append([float(lineArr[0]), float(lineArr[1])])

points = np.array(points)
tri = Delaunay(points)
center = np.sum(points[tri.simplices], axis=1) / 3.0


plt.triplot(points[:,0], points[:, 1], tri.simplices.copy(), linewidth=1.5)
plt.tick_params(labelbottom=False, labelleft=False, left=False, right=False, bottom=False, top=False)
ax = plt.gca()
plt.scatter(points[:, 0], points[:, 1], color='r')
# plt.grid()
plt.savefig('Delaunay.png', transparent=True, dpi=600)
plt.show()
