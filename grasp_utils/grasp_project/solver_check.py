import taichi as ti
import numpy as np

ti.init(arch=ti.cpu)  # Initialize Taichi with GPU support

# Parameters
num_points_observed = 10000  # Number of points in vector A
X_np = np.random.rand(num_points_observed, 3).astype(float)
y_np = np.ones((num_points_observed, 1))

# Define the Taichi data structures
X = ti.Vector.field(3, dtype=ti.f32, shape=num_points_observed)
y = ti.Vector.field(1, dtype=ti.f32, shape=num_points_observed)
K = ti.field(dtype=ti.f32, shape=(num_points_observed, num_points_observed))
X.from_numpy(X_np)
y.from_numpy(y_np)

l = 0.1
noise = 0.05


# Compute the distance matrix
@ti.kernel
def compute_K():
    for i in range(num_points_observed):
        for j in range(num_points_observed):
            K[i, j] = ti.math.distance(X[i], X[j])
            K[i, j] = ti.math.exp(-K[i, j] / l)
            if i == j:
                K[i, j] += noise


# Run the Taichi kernels
compute_K()

K_np = K.to_numpy()

import time
import numpy as np
import scipy
import torch
import jax.numpy as jnp

print('start!')

start = time.time()
model = scipy.linalg.solve(K_np, y_np)
print(time.time() - start)

start = time.time()
model_np= np.linalg.solve(K_np, y_np)
print(time.time() - start)

start = time.time()
model = torch.linalg.solve(torch.from_numpy(K_np.astype(float)), torch.from_numpy(y_np.astype(float)))
print(time.time() - start)


@torch.jit.script
def solver(K, y):
    return torch.linalg.solve(K, y)


start = time.time()
model = solver(torch.from_numpy(K_np.astype(float)), torch.from_numpy(y_np.astype(float)))
print(time.time() - start)

import math
import george
from george import kernels
kernel = kernels.ExpKernel(l**2, ndim=3)
gp = george.GP(kernel, solver=george.HODLRSolver)

start = time.time()
gp.compute(X_np, np.ones(num_points_observed) * 0.05)
model = gp.apply_inverse(y_np)
print(time.time() - start)