import numpy as np
import matplotlib.pyplot as plt
from pytransform3d.rotations import plot_basis
import pytransform3d.trajectories
from tqdm.auto import tqdm


def probablility(x, mu, sigma):
    mu_inv = pytransform3d.trajectories.invert_transforms(mu)
    log_mu = mu_inv[:, np.newaxis, :, :] @ x
    log_mu = pytransform3d.trajectories.exponential_coordinates_from_transforms(log_mu)
    N = (-3 * np.log(2 * np.pi) - 0.5 * np.log(np.linalg.det(sigma))[:, np.newaxis]
         - 0.5 * np.sum(log_mu @ np.linalg.inv(sigma) * log_mu, axis=2))
    N = np.exp(N)
    return N, log_mu


def dist(x, mu):
    mu_inv = pytransform3d.trajectories.invert_transforms(mu)
    log_mu = mu_inv[:, np.newaxis, :, :] @ x
    log_mu = pytransform3d.trajectories.exponential_coordinates_from_transforms(log_mu)
    return np.linalg.norm(log_mu, axis=-1)


def kmeanspp(x, n_clusters=64):
    index = np.random.choice(x.shape[0], 1, replace=False)
    mu = x[index]

    for i in tqdm(range(n_clusters - 1)):
        new_index = np.argmax(np.min(dist(x, mu), axis=0))
        mu = np.concatenate([mu, x[[new_index]]], axis=0)
    return mu


def step(x, mu, sigma, pi):
    N, log_mu = probablility(x, mu, sigma)
    r = N * pi
    r = r / np.sum(r, axis=0, keepdims=True)

    pi_new = np.mean(r, axis=1, keepdims=True)
    u_new = np.sum(r[:, :, np.newaxis] * log_mu, axis=1) / np.sum(r, axis=1, keepdims=True)
    mu_new = mu @ pytransform3d.trajectories.transforms_from_exponential_coordinates(u_new)
    log_mu = log_mu[:, :, :, np.newaxis]
    sigma_new = (np.sum(r[:, :, np.newaxis, np.newaxis] * log_mu @ np.swapaxes(log_mu, 2, 3), axis=1)
                 / np.sum(r, axis=1, keepdims=True)[:, :, np.newaxis])

    return mu_new, sigma_new, pi_new


def eval(x, mu, sigma, pi):
    N, log_mu = probablility(x, mu, sigma)
    prob = np.sum(N * pi, axis=0)
    return prob


# parameter = np.load('gmm_model_50.npz')
# mu = parameter['mu']
# sigma = parameter['sigma']
# pi = parameter['pi']
# data = np.load('data/ee_data100000.npz')
data = np.load('data_merged.npz')
x = data['data']
print(x.shape)

test_cases = [i for i in range(1, 71)]
# test_cases = [64]
results = []
for n_clusters in tqdm(test_cases, position=0, desc="clusters", leave=True, colour='green', ncols=160):
    iterations = 10

    # parameter = np.load('gmm_results/gmm_model_64_1000.npz')
    # mu = parameter['mu']
    # sigma = parameter['sigma']
    # pi = parameter['pi']

    index = np.random.choice(x.shape[0], 100000, replace=False)
    mu = kmeanspp(x[index], n_clusters)
    sigma = np.eye(6)[None, :, :].repeat(n_clusters, axis=0) * 0.1
    pi = np.ones((n_clusters, 1)) / n_clusters

    k = n_clusters * (6 + 6 ** 2)
    BIC_best = np.inf

    bar = tqdm(range(iterations), position=1, desc="iterations", leave=False, colour='red', ncols=160)
    for i in bar:
        index = np.random.choice(x.shape[0], 100000, replace=False)
        x_batch = x[index]
        n = x_batch.shape[0]

        mu, sigma, pi = step(x_batch, mu, sigma, pi)
        prob = eval(x_batch, mu, sigma, pi)
        L = np.log(prob)
        BIC = k * np.log(float(n)) - 2 * L.sum()
        BIC_best = np.min([BIC_best, BIC])
        bar.set_description("iterations (BIC: %6.0f, prob: %1.3f)" % (BIC, prob.mean()))

        # if i % 100 == 0:
        #     np.savez('gmm_results/gmm_model_' + str(n_clusters) + '_' + str(iterations) + '.npz',
        #              mu=mu, sigma=sigma, pi=pi)

    results.append(BIC_best)


# ax = None
# for t in range(len(mu)):
#     ax = plot_basis(ax=ax, s=0.15, R=mu[t, :3, :3], p=mu[t, :3, 3])
print(test_cases, results)
plt.plot(test_cases, results)
plt.show()
