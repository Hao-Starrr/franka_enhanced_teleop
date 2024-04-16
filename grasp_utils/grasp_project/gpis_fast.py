import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

l = 1
noise = 0.05


# sampling the surface of a sphere
def fibonacci_sphere(num_samples):
    phi = np.pi * (3. - np.sqrt(5.))  # golden angle in radians
    y = np.linspace(1, -1, num_samples)
    radius = np.sqrt(1 - y * y)
    theta = phi * np.arange(num_samples)
    x = np.cos(theta) * radius
    z = np.sin(theta) * radius
    return x, y, z


def reverting_function(x):
    return - l * np.log(x)


def reverting_function_derivative(x):
    return -l / x


if __name__ == "__main__":
    # creating a sphere point cloud
    N_obs = 1000  # number of observations
    sphereRadius = 1
    xa, yb, zc = fibonacci_sphere(N_obs)
    sphere = sphereRadius * np.concatenate([xa.reshape(-1, 1), yb.reshape(-1, 1), zc.reshape(-1, 1)], axis=1)

    # using a 2D plane to query the distances
    start = -2
    end = 2
    samples = int((end - start) / 0.05) + 1
    xg, yg = np.meshgrid(np.linspace(start, end, samples), np.linspace(start, end, samples));
    querySlice = np.concatenate([xg.reshape(-1, 1), yg.reshape(-1, 1), np.zeros(xg.shape).reshape(-1, 1)], axis=1)

    import george
    from george import kernels
    import time

    start = time.time()
    kernel = kernels.ExpKernel(l ** 2, ndim=3)
    gp = george.GP(kernel)

    gp.compute(sphere, np.ones(N_obs) * 0.05)
    model = gp.apply_inverse(np.ones((N_obs, 1)))
    k = gp.get_matrix(querySlice, sphere)

    # distance inference
    mu = k @ model
    mean = reverting_function(mu)
    mu_derivative = np.moveaxis(kernel.get_x1_gradient(querySlice, sphere), -1, 0) @ model
    grad = reverting_function_derivative(mu) * mu_derivative

    # gradient normalization
    norms = np.linalg.norm(grad, axis=0, keepdims=True)
    grad = np.where(norms != 0, grad, grad / np.min(np.abs(grad), axis=0))
    grad /= np.linalg.norm(grad, axis=0, keepdims=True)

    dist = mean
    mean_orig = np.copy(mean)
    query_new = np.copy(querySlice)
    for i in range(5):
        query_new -= mean * grad[:, :, 0].T
        k = gp.get_matrix(query_new, sphere)
        mu = k @ model
        mean = reverting_function(mu)
        dist += mean
    mean = dist
    print(time.time() - start)

    mean_real = np.linalg.norm(querySlice, axis=1)

    # GPIS visualization
    xg = querySlice[:, 0].reshape(xg.shape)
    yg = querySlice[:, 1].reshape(yg.shape)
    zg = np.zeros(xg.shape)

    xd = grad[0].reshape(xg.shape)
    yd = grad[1].reshape(yg.shape)
    zd = grad[2].reshape(xg.shape)
    colors = np.arctan2(xd, yd)

    fig = plt.figure(figsize=(15, 5))
    fig.suptitle('GPIS result')

    # First subplot
    ax = fig.add_subplot(1, 3, 1, projection='3d')
    surf = ax.plot_surface(xg, yg, mean.reshape(xg.shape), cmap=cm.coolwarm,
                           linewidth=0, antialiased=False)
    plt.colorbar(surf, shrink=0.5, aspect=5, ax=ax)
    ax.set_title('Distance field')

    ax = fig.add_subplot(1, 3, 2, projection='3d')
    surf = ax.plot_surface(xg, yg, mean_orig.reshape(xg.shape), cmap=cm.coolwarm,
                           linewidth=0, antialiased=False)
    plt.colorbar(surf, shrink=0.5, aspect=5, ax=ax)
    ax.set_title('Distance field')

    # Second subplot
    ax = fig.add_subplot(1, 3, 3)
    colormap = cm.inferno
    ax.scatter(sphere[:, 0], sphere[:, 1], alpha=0.05)
    ax.quiver(xg, yg, xd, yd, colors, angles='xy', scale=100)
    ax.set_aspect('equal')
    ax.set_title('Gradient field')

    plt.show()
