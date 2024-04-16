import csv
import open3d as o3d
import numpy as np
from gpis import RGPIS
from tqdm.auto import tqdm
import matplotlib.pyplot as plt


def openfile(filename="datasets.csv"):
    with open(filename, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        files = []
        for lines in csvreader:
            files.append(lines)
    return files


files = openfile()
n = len(files)
test_cases = np.arange(0.02, 0.62, 0.02) # [0.01, 0.02, 0.04, 0.08, 0.16, 0.32, 0.64, 1.28]
results = []

iterator_first = tqdm(test_cases, position=0, desc="parameter", leave=True, colour='green', ncols=160)
for parameter in iterator_first:
    gp = RGPIS(l=parameter)
    iterator_second = tqdm(range(n), position=1, desc="files", leave=False, colour='red', ncols=160)
    chamfer_dist = 0
    for idx in iterator_second:
        pcd = o3d.io.read_point_cloud(files[idx][1][3:])

        camera_view = np.random.rand(3)
        camera_view /= np.linalg.norm(camera_view)
        radius = 100.0
        _, pt_map = pcd.hidden_point_removal(camera_view, radius)
        pcd_partial = pcd.select_by_index(pt_map)

        pcd = pcd.voxel_down_sample(voxel_size=0.02)
        pcd_partial = pcd_partial.voxel_down_sample(voxel_size=0.02)

        pcd = np.asarray(pcd.points)
        pcd_partial = np.asarray(pcd_partial.points)

        gp.train(pcd_partial)
        d = gp.ray_marching(pcd, distance=True)
        chamfer_dist += np.abs(d).mean()

    results.append(chamfer_dist / n)

print(test_cases)
print(results)
plt.plot(test_cases, results)
plt.show()