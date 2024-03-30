import matplotlib.pyplot as plt
import numpy as np
from mayavi import mlab
import open3d as o3d
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors
import pyvista as pv
from scipy.signal import savgol_filter
from statsmodels.nonparametric.kernel_regression import KernelReg

def smooth(y, box_pts):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y, box, mode='same')
    return y_smooth

file_3d = open("data.txt", "r")

robot_pos = []
x = []
for i in file_3d.readlines():
    if "position" in i:
        x = []
    elif "x:" in i:
        x.append(float(i.split(":")[1][1:]))
    elif "y:" in i:
        x.append(float(i.split(":")[1][1:]))
    elif "z:" in i:
        x.append(float(i.split(":")[1][1:]))
        # x.append(1)
        robot_pos.append(x)

robot_pos = np.array(robot_pos)
robot_pos = robot_pos[:, :3]
robot_pos = np.unique(robot_pos, axis=0)

# file_3d.close()
# print(robot_pos.shape)
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(robot_pos[:, :3])
# pcd.colors = o3d.utility.Vector3dVector(np.random.randint(255, size=robot_pos[:, :3].shape))
# pcd.normals = o3d.utility.Vector3dVector(np.ones(robot_pos[:, :3].shape)*0.1)
#
# o3d.visualization.draw_geometries([pcd])
# distances = pcd.compute_nearest_neighbor_distance()
# avg_dist = np.mean(distances)
# radius = 3 * avg_dist

# with o3d.utility.VerbosityContextManager(
#         o3d.utility.VerbosityLevel.Debug) as cm:
#     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
#         pcd, depth=9)
# print(mesh)
# o3d.visualization.draw_geometries([mesh],
#                                   zoom=0.664,
#                                   front=[-0.4761, -0.4698, -0.7434],
#                                   lookat=[1.8900, 3.2596, 0.9284],
#                                   up=[0.2304, -0.8825, 0.4101])

# print('visualize densities')
# densities = np.asarray(densities)
# density_colors = plt.get_cmap('plasma')(
#     (densities - densities.min()) / (densities.max() - densities.min()))
# density_colors = density_colors[:, :3]
# density_mesh = o3d.geometry.TriangleMesh()
# density_mesh.vertices = mesh.vertices
# density_mesh.triangles = mesh.triangles
# density_mesh.triangle_normals = mesh.triangle_normals
# density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
# o3d.visualization.draw_geometries([density_mesh],
#                                   zoom=0.664,
#                                   front=[-0.4761, -0.4698, -0.7434],
#                                   lookat=[1.8900, 3.2596, 0.9284],
#                                   up=[0.2304, -0.8825, 0.4101])

# radii = [0.005, 0.01, 0.02, 0.04]
# rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
#     pcd, o3d.utility.DoubleVector(radii))
# o3d.visualization.draw_geometries([pcd, rec_mesh])


robot_pos[:, 2] = smooth(robot_pos[:, 2],2251)

# kr = KernelReg(robot_pos[:, 2],robot_pos[:, 0],'c')
# robot_pos[:, 2], y_std = kr.fit(robot_pos[:, 0])


nbrs = NearestNeighbors(n_neighbors = 5)
nbrs.fit(robot_pos)
distances, indexes = nbrs.kneighbors(robot_pos)
plt.plot(distances.mean(axis =1))
plt.show()

outlier_index = np.where(distances.mean(axis = 1) > 0.000015)
print(len(outlier_index))
print(robot_pos.shape)
robot_pos = np.delete(robot_pos, outlier_index, 0)
print(robot_pos.shape)


X = robot_pos[:, 0]
Y = robot_pos[:, 1]
Z = robot_pos[:, 2]
Z[Z >= 0.008] = 0.008
Z[Z <= -0.07] = -0.07
pts = mlab.points3d(X, Y, Z, Z)
mesh = mlab.pipeline.delaunay2d(pts)
pts.remove()
surf = mlab.pipeline.surface(mesh)
mlab.xlabel("x")
mlab.ylabel("y")
mlab.zlabel("z")
mlab.show()
print(max(Z))


# cloud = pv.PolyData(robot_pos)
# cloud.plot()
#
# volume = cloud.delaunay_3d(alpha=2.)
# shell = volume.extract_geometry()
# shell.plot()