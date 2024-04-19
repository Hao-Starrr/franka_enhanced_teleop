import numpy as np
import george
import copy


class RGPIS:
    def __init__(self, l=0.3, noise=0.05, dim=3):
        self.l = l
        self.noise = noise
        self.kernel = george.kernels.ExpKernel(l ** 2, ndim=dim)
        self.gp = george.GP(self.kernel, solver=george.BasicSolver)
        self.model = None
        self.X = None

    def train(self, X):
        n_obs = X.shape[0]
        self.X = X
        self.gp.compute(X, np.ones(n_obs) * 0.05)
        self.model = self.gp.apply_inverse(np.ones((n_obs, 1)))

    def occupancy_check(self, x):
        k = self.gp.get_matrix(x, self.X)
        mu = k @ self.model
        return mu > np.exp(-0.002 / self.l)

    def reverting_function(self, x):
        return - self.l * np.log(x + 1e-6)

    def reverting_function_derivative(self, x):
        return - self.l / x

    def get_distance(self, x):
        k = self.gp.get_matrix(x, self.X)
        mu = k @ self.model
        return self.reverting_function(mu)

    def get_surface_normal(self, x):
        k = self.gp.get_matrix(x, self.X)
        mu = k @ self.model
        mu_derivative = np.moveaxis(self.kernel.get_x1_gradient(x, self.X), -1, 0) @ self.model
        grad = self.reverting_function_derivative(mu) * mu_derivative
        norms = np.linalg.norm(grad, axis=0, keepdims=True)
        grad = np.where(norms != 0, grad, grad / np.min(np.abs(grad), axis=0))
        grad /= np.linalg.norm(grad, axis=0, keepdims=True)
        grad = -grad[:, :, 0].T
        return grad

    def ray_marching(self, x, iterations=5, distance=False):
        k = self.gp.get_matrix(x, self.X)
        mu = k @ self.model
        mean = self.reverting_function(mu)
        mu_derivative = np.moveaxis(self.kernel.get_x1_gradient(x, self.X), -1, 0) @ self.model
        grad = self.reverting_function_derivative(mu) * mu_derivative

        norms = np.linalg.norm(grad, axis=0, keepdims=True)
        grad = np.where(norms != 0, grad, grad / np.min(np.abs(grad), axis=0))
        grad /= np.linalg.norm(grad, axis=0, keepdims=True)
        grad = -grad[:, :, 0].T

        if distance:
            mean_sum = copy.deepcopy(mean)

        for i in range(iterations - 1):
            x += mean * grad
            k = self.gp.get_matrix(x, self.X)
            mu = k @ self.model
            mean = self.reverting_function(mu)
            if distance:
                mean_sum += mean

        if distance:
            return mean_sum

        return x, grad

    def get_surface_points(self, samples=8):
        x_min, x_max = self.X[:, 0].min(), self.X[:, 0].max()
        y_min, y_max = self.X[:, 1].min(), self.X[:, 1].max()
        z_min, z_max = self.X[:, 2].min(), self.X[:, 2].max()
        x_diff, y_diff, z_diff = (x_max - x_min) / 2, (y_max - y_min) / 2, (z_max - z_min) / 2
        # x_diff, y_diff, z_diff = 0, 0, 0

        xg, yg, zg = np.meshgrid(np.linspace(x_min - x_diff, x_max + x_diff, samples),
                                 np.linspace(y_min - y_diff, y_max + y_diff, samples),
                                 np.linspace(z_min - z_diff, z_max + z_diff, samples * 3))

        x = np.concatenate([xg.reshape(-1, 1), yg.reshape(-1, 1), zg.reshape(-1, 1)], axis=1)
        return self.ray_marching(x, iterations=10)

    def get_var(self, x):
        kxx = self.gp.get_matrix(x, x)
        kxX = self.gp.get_matrix(x, self.X)
        return kxx - kxX @ self.gp.apply_inverse(kxX.T)