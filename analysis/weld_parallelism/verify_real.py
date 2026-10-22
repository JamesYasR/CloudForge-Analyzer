#!/usr/bin/env python3
import hashlib
import importlib.util
import json
import pathlib
import sys

import numpy as np

root = pathlib.Path(__file__).resolve().parents[2]
folder = root / 'analysis/weld_parallelism'
source = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else '/media/jamesyasr/Shared/大创/演示案例/3_焊缝平行度')
spec = importlib.util.spec_from_file_location('pcd_reader', root / 'analysis/cylinder_patch/analyze_cylinder_patch.py')
reader = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reader)
reference = dict(line.split(' ', 1) for line in (folder / 'reference_real.txt').read_text().splitlines())
assert reference['weld_constraint'] == '1'
coeff = np.fromstring(reference['final'], sep=' ')
origin, axis, radius = coeff[:3], coeff[3:6], coeff[6]
axis /= np.linalg.norm(axis)
helper = np.eye(3)[np.argmin(np.abs(axis))]
v = helper - np.dot(helper, axis) * axis
v /= np.linalg.norm(v)
w = np.cross(axis, v)
w /= np.linalg.norm(w)
v = np.cross(w, axis)
clouds = [reader.read_pcd_xyz(str(source / f'焊缝{i}.pcd'))[0].astype(float) for i in (1, 2)]
phis = [np.arctan2((p-origin) @ w, (p-origin) @ v) for p in clouds]
all_phi = np.concatenate(phis)
phi0 = np.arctan2(np.sin(all_phi).sum(), np.cos(all_phi).sum())
projected = [np.column_stack((radius*((phi-phi0+np.pi) % (2*np.pi)-np.pi), (p-origin) @ axis)) for p, phi in zip(clouds, phis)]
export = np.genfromtxt(folder / 'projected_real.csv', delimiter=',', names=True)
projection_error = max(float(np.max(np.abs(p-np.column_stack((export['s'][export['seam']==i], export['a'][export['seam']==i]))))) for i, p in enumerate(projected))
assert projection_error < 1e-8, projection_error

def tls(points, weights=None):
    if weights is None:
        weights = np.ones(len(points))
    center = np.average(points, axis=0, weights=weights)
    _, _, basis = np.linalg.svd((points-center)*np.sqrt(weights[:, None]), full_matrices=False)
    return center, basis[0]

def seed(points):
    center = np.median(points, axis=0)
    distances = np.linalg.norm(points-center, axis=1)
    return tls(points[distances <= max(4*np.median(distances), 5)])[1]

u, z = map(seed, projected)
if np.dot(u, z) < 0:
    z = -z
along = u + z
along /= np.linalg.norm(along)
across = np.array([-along[1], along[0]])
coordinates = [np.column_stack((p@along, p@across)) for p in projected]
low = max(np.quantile(p[:, 0], .02) for p in coordinates)
high = min(np.quantile(p[:, 0], .98) for p in coordinates)
count = int(np.clip((high-low)//2, 6, 200))
step = (high-low)/count
indices = [np.floor((p[:, 0]-low)/step).astype(int) for p in coordinates]
valid = [i for i in range(count) if all(np.sum(ids==i)>=3 for ids in indices)]
groups = np.split(np.array(valid), np.where(np.diff(valid)>3)[0]+1)
chosen = max(groups, key=len)
centers, lines = [], []
for points, initial in zip(projected, (u, z)):
    direction = initial.copy()
    for _ in range(3):
        normal = np.array([-direction[1], direction[0]])
        offset = np.median(points@normal)
        start, end = sorted(((low+chosen[0]*step-along@normal*offset)/(along@direction),
                             (low+(chosen[-1]+1)*step-along@normal*offset)/(along@direction)))
        ids = np.floor((points@direction-start)/((end-start)/len(chosen))).astype(int)
        local = np.column_stack((points@direction, points@normal))
        samples = np.array([np.median(local[ids==i], axis=0) for i in range(len(chosen)) if np.sum(ids==i)>=3])
        samples = samples@np.array([direction, normal])
        weights = np.ones(len(samples))
        for _ in range(5):
            center, direction = tls(samples, weights)
            residual = np.abs(direction[0]*(samples[:,1]-center[1])-direction[1]*(samples[:,0]-center[0]))
            cutoff = max(.05, 3*1.4826*np.median(residual))
            weights = np.minimum(1, cutoff/np.maximum(residual, 1e-300))
    centers.append(samples)
    lines.append(direction)
angle = np.degrees(np.arctan2(abs(lines[0][0]*lines[1][1]-lines[0][1]*lines[1][0]), abs(np.dot(*lines))))
result = (folder / 'result_real.txt').read_text().splitlines()
actual = float(result[0].split()[1])
length = (chosen[-1]-chosen[0]+1)*step
length_error = abs(float(result[1].split()[1])-length)
assert abs(angle-actual)<1e-8, (angle, actual)
assert length_error<1e-8, length_error
fingerprints = {name: hashlib.sha256((source/name).read_bytes()).hexdigest() for name in ('未处理的点云.pcd', '焊缝1.pcd', '焊缝2.pcd', '圆柱壁面.pcd')}
manifest = {
    'reference_source': '未处理的点云.pcd', 'sha256': fingerprints,
    'design_radius_mm': 1940, 'weld_constraint_applied': True,
    'fit_parameters': {'normal_neighbors': 60, 'ransac_distance_mm': 3, 'ransac_iterations': 1000, 'optimization_iterations': 2000, 'tolerance_mm': 1, 'weld_threshold_factor': 3, 'weld_cluster_tolerance_mm': 2, 'weld_min_cluster_size': 20, 'weld_constraint_weight': 1},
    'independent_angle_degrees': float(angle), 'comparison_length_mm': float(length),
    'angle_difference_degrees': float(abs(angle-actual)), 'projection_max_difference_mm': projection_error,
    'length_difference_mm': float(length_error), 'common_center_count': len(chosen),
}
(folder / 'independent_real.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
print(json.dumps(manifest, ensure_ascii=False, indent=2))
