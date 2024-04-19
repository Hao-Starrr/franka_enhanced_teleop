import numpy as np
import pytransform3d.trajectories


class SE3GMM:
    def __init__(self):
        self.n_clusters = None
        self.mu = None
        self.sigma = None
        self.pi = None

    def prob(self, x):
        mu_inv = pytransform3d.trajectories.invert_transforms(self.mu)
        log_mu = mu_inv[:, np.newaxis, :, :] @ x
        log_mu = pytransform3d.trajectories.exponential_coordinates_from_transforms(log_mu)
        self.sigma[:, np.arange(6), np.arange(6)] += 1e-6
        N = (-3 * np.log(2 * np.pi) - 0.5 * np.log(np.linalg.det(self.sigma))[:, np.newaxis]
             - 0.5 * np.sum(log_mu @ np.linalg.inv(self.sigma) * log_mu, axis=2))
        N = np.exp(N)
        return N, log_mu

    def eval_temp(self, x):
        mu_inv = pytransform3d.trajectories.invert_transforms(self.mu)
        log_mu = mu_inv[:, np.newaxis, :, :] @ x
        log_mu = pytransform3d.trajectories.exponential_coordinates_from_transforms(log_mu)
        self.sigma[:, np.arange(6), np.arange(6)] += 1e-6
        N = (-3 * np.log(2 * np.pi) - 0.5 * np.log(np.linalg.det(self.sigma))[:, np.newaxis]
             - 0.5 * np.sum(log_mu @ np.linalg.inv(self.sigma) * log_mu, axis=2))
        N_max = np.max(N)
        prob_x = np.sum(np.exp(N-N_max) * self.pi, axis=0)
        return prob_x

    def grad(self, x):
        eps = 1e-6
        X = np.stack([x,
                      x + np.array([eps, 0, 0, 0, 0, 0]),
                      x + np.array([0, eps, 0, 0, 0, 0]),
                      x + np.array([0, 0, eps, 0, 0, 0]),
                      x + np.array([0, 0, 0, eps, 0, 0]),
                      x + np.array([0, 0, 0, 0, eps, 0]),
                      x + np.array([0, 0, 0, 0, 0, eps])], axis=0)
        X = pytransform3d.trajectories.transforms_from_exponential_coordinates(X)
        prob = self.eval_temp(X)
        grad = (prob[1:8] - prob[0:1]) / eps
        return grad / (prob[0] + 1e-12)

    def em_step(self, x):
        N, log_mu = self.prob(x)
        r = N * self.pi
        r = r / (np.sum(r, axis=0, keepdims=True) + 1e-6)

        pi_new = np.mean(r, axis=1, keepdims=True)
        u_new = np.sum(r[:, :, np.newaxis] * log_mu, axis=1) / np.sum(r, axis=1, keepdims=True)
        mu_new = self.mu @ pytransform3d.trajectories.transforms_from_exponential_coordinates(u_new)
        log_mu = log_mu[:, :, :, np.newaxis]
        sigma_new = (np.sum(r[:, :, np.newaxis, np.newaxis] * log_mu @ np.swapaxes(log_mu, 2, 3), axis=1)
                     / np.sum(r, axis=1, keepdims=True)[:, :, np.newaxis])

        self.mu = mu_new
        self.sigma = sigma_new
        self.pi = pi_new

    def eval(self, x):
        N, log_mu = self.prob(x)
        prob_x = np.sum(N * self.pi, axis=0)
        return prob_x

    def BIC(self, x):
        k = self.n_clusters * (6 + 6 ** 2)
        n = x.shape[0]

        prob = self.eval(x)
        L = np.log(prob)
        return k * np.log(float(n)) - 2 * L.sum()

    def fit(self, x, n_clusters=3, n_iterations=100, batch_size=None, show=False, sample=True):
        self.n_clusters = n_clusters
        index = np.random.choice(x.shape[0], self.n_clusters, replace=False)
        if batch_size is None:
            batch_size = x.shape[0]
        self.mu = x[index]
        self.sigma = np.eye(6)[None, :, :].repeat(self.n_clusters, axis=0) * 0.1
        self.pi = np.ones((self.n_clusters, 1)) / self.n_clusters

        for i in range(n_iterations):
            index = np.random.choice(x.shape[0], batch_size, replace=False)
            x_batch = x[index]
            self.em_step(x_batch)
            if show:
                print(self.BIC(x_batch))

        if sample:
            N, _ = self.prob(x)
            samples = np.unique(np.argmax(N, axis=1))
            return x[samples]
        else:
            return
