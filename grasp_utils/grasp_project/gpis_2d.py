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
    start = -5
    end = 5
    samples = int((end - start) / 0.05) + 1
    xg = np.linspace(start, end, samples)[:, np.newaxis]
    querySlice = np.concatenate([xg, np.zeros(xg.shape), np.zeros(xg.shape)], axis=1)

    import george
    from george import kernels

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

    mean_real = np.linalg.norm(querySlice, axis=1) - 1

    plt.rcParams['text.usetex'] = True
    fig = plt.figure(figsize=(8, 5))

    # First subplot
    # ax = fig.add_subplot(1, 1, 1)
    # ax.plot(xg, mean_real, '-k', label='real distance field')
    # ax.scatter([-1, 1], [0, 0], c='k', label='surface points')
    # ax.plot(xg, mean_orig, '--r', label='estimated distance field')
    # ax.grid(True)
    # ax.set_xlim([start, end])
    # ax.set_xlabel(r'$x$ [m]')
    # ax.set_ylabel('signed distance (m)')
    # ax.set_title('Distance field of a sphere (raw)')
    # leg = ax.legend(loc="upper center")

    ax = fig.add_subplot(1, 1, 1)
    ax.plot(xg, mean_real, '-k', label='real distance field')
    ax.scatter([-1, 1], [0, 0], c='k', label='surface points')
    ax.plot(xg, mean, '--r', label='estimated distance field')
    ax.grid(True)
    ax.set_xlim([start, end])
    ax.set_xlabel(r'$x$ [m]')
    ax.set_ylabel('signed distance (m)')
    ax.set_title('Distance field of a sphere (refined)')
    leg = ax.legend(loc="upper center")

    plt.tight_layout()
    plt.show()

    # import tikzplotlib
    #
    # tikzplotlib.save("mytikz.tex")