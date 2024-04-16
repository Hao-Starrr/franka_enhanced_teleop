import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
plt.rcParams['text.usetex'] = True
link_lengths = np.array([3, 2])
n_dof = len(link_lengths)


def fk(joint_positions):
    thetas = np.cumsum(joint_positions)
    xs = np.cumsum(np.cos(thetas) * link_lengths)
    ys = np.cumsum(np.sin(thetas) * link_lengths)
    return xs, ys, thetas


def Jacobian(joint_positions):
    thetas = np.cumsum(joint_positions)
    J_x = np.cumsum(-(np.sin(thetas) * link_lengths)[::-1])[::-1]
    J_y = np.cumsum((np.cos(thetas) * link_lengths)[::-1])[::-1]
    J_theta = np.ones(len(joint_positions))
    return np.stack([J_x, J_y, J_theta], axis=0)


def ik_2d(x, y):
    temp = (x ** 2 + y ** 2 - link_lengths[0] ** 2 - link_lengths[1] ** 2) / (2 * link_lengths[0] * link_lengths[1])
    q2_d = np.arccos(temp)
    q1_d = np.arctan2(y, x) - np.arctan2(
        link_lengths[1] * np.sin(q2_d), (link_lengths[0] + link_lengths[1] * np.cos(q2_d)))
    q2_u = -np.arccos(temp)
    q1_u = np.arctan2(y, x) - np.arctan2(
        link_lengths[1] * np.sin(q2_u), (link_lengths[0] + link_lengths[1] * np.cos(q2_u)))
    return np.array([[q1_d, q2_d],
                     [q1_u, q2_u]])


fig, axs = plt.subplots(1, 1)
current_joint = np.random.uniform(-np.pi, np.pi, size=(2,))
current_joint = np.array([0, 0])
target_joints = np.empty((0, 2))
thetas = np.linspace(-np.pi, np.pi, 30)

drawings = []
# target_poses = []
for theta in thetas:
    # target_pose = np.array([2 * np.cos(theta), 1 * np.sin(theta)])
    target_pose = np.array([16 * np.sin(theta) ** 3,
                            13 * np.cos(theta) - 5 * np.cos(2 * theta) - 2 * np.cos(3 * theta) - np.cos(
                                4 * theta)]) * 0.2
    target_joint = ik_2d(target_pose[0], target_pose[1])
    target_joints = np.concatenate([target_joints, target_joint], axis=0)

    for target in target_joint:
        xs, ys, _ = fk(target)

        joint_origin = np.zeros(2)
        for i in range(n_dof):
            # drawings.append(axs[0].plot([joint_origin[0], xs[i]], [joint_origin[1], ys[i]], 'o-k'))
            # drawings.append(axs.plot([joint_origin[0], xs[i]], [joint_origin[1], ys[i]], 'o-k'))
            joint_origin = np.array([xs[i], ys[i]])

    # drawings.append(axs[0].scatter(target_pose[0], target_pose[1], c='r', zorder=3))
    # drawings.append(axs[1].scatter(target_joint[:, 0], target_joint[:, 1], c='k', zorder=0))

    # drawings.append(axs.scatter(target_pose[0], target_pose[1], c='r', zorder=3))
    # target_poses.append(target_pose)
    # drawings.append(axs.scatter(target_joint[:, 0], target_joint[:, 1], c='k', zorder=0))
    # target_jointss.append(target_joint)
# target_poses = np.stack(target_poses)
# axs.scatter(target_poses[:, 0], target_poses[:, 1], c='r', zorder=3)
axs.scatter(target_joints[:, 0], target_joints[:, 1], c='k', zorder=0)

from gpis import RGPIS
gp = RGPIS(l=1, dim=2)
gp.train(target_joints)

start = -np.pi
end = np.pi
samples = int((end - start) / 0.01) + 1
xg, yg = np.meshgrid(np.linspace(start, end, samples), np.linspace(start, end, samples))
x = np.column_stack((xg.reshape(-1), yg.reshape(-1)))
import copy
_, grad = gp.ray_marching(copy.deepcopy(x))

xg = x[:, 0].reshape(xg.shape)
yg = x[:, 1].reshape(yg.shape)

xd = grad[:, 0].reshape(xg.shape)
yd = grad[:, 1].reshape(yg.shape)
colors = np.arctan2(xd, yd)

# axs[0].set_xlim([-6, 6])
# axs[0].set_ylim([-6, 6])
# axs[0].set_aspect('equal')
# axs[0].grid()
# axs[0].set_title('Target poses and IK')
# axs[0].set_xlabel('x')
# axs[0].set_ylabel('y')

# axs.set_xlim([-6, 6])
# axs.set_ylim([-6, 6])
# axs.set_aspect('equal')
# axs.grid()
# axs.set_title('Target poses and IK')
# axs.set_xlabel('x')
# axs.set_ylabel('y')

# axs[1].set_xlim([-np.pi, np.pi])
# axs[1].set_ylim([-np.pi, np.pi])
# axs[1].set_aspect('equal')
# axs[1].grid()
# axs[1].quiver(xg, yg, xd, yd, colors, angles='xy', scale=100, cmap='hsv', zorder=-1)
# axs[1].set_title('Joint space')
# axs[1].set_xlabel('q1')
# axs[1].set_ylabel('q2')
#
axs.set_xlim([-np.pi, np.pi])
axs.set_ylim([-np.pi, np.pi])
axs.set_aspect('equal')
axs.grid()
# axs.quiver(xg, yg, xd, yd, colors, angles='xy', scale=100, cmap='hsv', zorder=-1)
axs.imshow(colors[::-1, :], cmap='hsv', extent=[-np.pi, np.pi, -np.pi, np.pi], zorder=-1)
axs.set_title(r'Joint space ($l=1$)')
axs.set_xlabel('q1')
axs.set_ylabel('q2')

plt.show()
# import tikzplotlib
# tikzplotlib.save("joint_plot.tex")