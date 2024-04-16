import numpy as np

from relie.utils.se3_tools import se3_exp, se3_log, se3_vee, se3_inv
import torch


def probablility(x, mu, sigma):
    log_mu = se3_inv(mu)[:, None, :, :] @ x
    print(log_mu.isnan().any(), log_mu.isinf().any())
    log_mu = se3_log(log_mu)
    print(log_mu.isnan().any(), log_mu.isinf().any())
    log_mu = se3_vee(log_mu)
    print(log_mu.isnan().any(), log_mu.isinf().any())
    L = torch.linalg.cholesky(sigma)
    print(L.isnan().any(), L.isinf().any())
    y = (torch.linalg.inv(L)[:, None, :, :] @ log_mu[:, :, :, None])[:, :, :, 0]
    N = (-3 * np.log(2 * torch.pi) - torch.log(torch.linalg.det(L))[:, None]
         - 0.5 * (y * y).sum(2))
    print(N.isnan().any(), N.isinf().any())
    N = torch.exp(N)
    return N, log_mu


def step(x, mu, sigma, pi):
    N, log_mu = probablility(x, mu, sigma)
    r = N * pi
    r = r / torch.sum(r, dim=0, keepdim=True)
    # print(N.isnan().any(), N.isinf().any())
    # print(log_mu.isnan().any(), log_mu.isinf().any())
    # print(r.isnan().any(), r.isinf().any())

    pi_new = torch.mean(r, dim=1, keepdim=True)
    u_new = torch.sum(r[:, :, None] * log_mu, dim=1) / torch.sum(r, dim=1, keepdim=True)
    mu_new = se3_exp(u_new)

    sigma_new = log_mu.transpose(1, 2) @ log_mu / log_mu.size(1)
    # print(pi_new.isnan().any(), pi_new.isinf().any())
    # print(u_new.isnan().any(), u_new.isinf().any())
    # print(mu_new.isnan().any(), mu_new.isinf().any())

    return mu_new, sigma_new, pi_new


def eval(x, mu, sigma, pi):
    N, log_mu = probablility(x, mu, sigma)
    prob = N * pi[:, None, :]
    prob = prob.sum(0)
    return prob


iterations = 100
n_clusters = 23
mus = torch.zeros((n_clusters, 4, 4))
sigmas = torch.zeros((n_clusters, 6, 6))
pis = torch.ones((n_clusters, 1)) / n_clusters

data = np.load('data/ee_data100000.npz')
R = data['Rs'].reshape((-1, 3, 3))
t = data['ts']
x = np.eye(4)[None, :, :].repeat(R.shape[0], axis=0)
x[:, :3, :3] = R
x[:, :3, 3] = t
x = x[:1000]

index = np.random.choice(x.shape[0], n_clusters, replace=False)
mu = x[index]
sigma = np.eye(6)[None, :, :].repeat(n_clusters, axis=0)
pi = np.ones((n_clusters, 1)) / n_clusters

x = torch.tensor(x)
mu = torch.tensor(mu)
sigma = torch.tensor(sigma)
pi = torch.tensor(pi)

n = x.size(0)
k = n_clusters * (4 ** 2 + 6 ** 2)

for i in range(iterations):
    mu, sigma, pi = step(x, mu, sigma, pi)
    prob = eval(x, mu, sigma, pi)
    L = prob.sum(0).mean(0)
    BIC = k * np.log(n) - 2 * torch.log(L)
    print(mu.isnan().any(), mu.isinf().any())
    print(sigma.isnan().any(), sigma.isinf().any())
    print(pi.isnan().any(), pi.isinf().any())
    print(BIC)
    # if i == 1:
    #     break

np.savez("gmm_model_50.npz", mus=mus, sigmas=sigmas, pis=pis)
