import numpy as np
import trimesh


def create_gripper_marker(color=[0, 0, 255, 255], tube_radius=0.001, sections=6, scale=1.):
    """Create a 3D mesh visualizing a parallel yaw gripper. It consists of four cylinders.

    Args:
        color (list, optional): RGB values of marker. Defaults to [0, 0, 255].
        tube_radius (float, optional): Radius of cylinders. Defaults to 0.001.
        sections (int, optional): Number of sections of each cylinder. Defaults to 6.

    Returns:
        trimesh.Trimesh: A mesh that represents a simple parallel yaw gripper.
    """
    cfl = trimesh.creation.cylinder(
        radius=0.002 * scale,
        sections=sections,
        segment=[
            [4.10000000e-02 * scale, -7.27595772e-12 * scale, 6.59999996e-02 * scale],
            [4.10000000e-02 * scale, -7.27595772e-12 * scale, 1.12169998e-01 * scale],
        ],
    )
    cfr = trimesh.creation.cylinder(
        radius=0.002 * scale,
        sections=sections,
        segment=[
            [-4.100000e-02 * scale, -7.27595772e-12 * scale, 6.59999996e-02 * scale],
            [-4.100000e-02 * scale, -7.27595772e-12 * scale, 1.12169998e-01 * scale],
        ],
    )
    cb1 = trimesh.creation.cylinder(
        radius=0.002 * scale, sections=sections, segment=[[0, 0, 0], [0, 0, 6.59999996e-02 * scale]]
    )
    cb2 = trimesh.creation.cylinder(
        radius=0.002 * scale,
        sections=sections,
        segment=[[-4.100000e-02 * scale, 0, 6.59999996e-02 * scale], [4.100000e-02 * scale, 0, 6.59999996e-02 * scale]],
    )

    tmp = trimesh.util.concatenate([cb1, cb2, cfr, cfl])
    tmp.visual.face_colors = color

    return tmp


def visualize_grasps(Hs, scale=1., p_cloud=None, c_cloud=None, energies=None, colors=None, mesh=None, show=True):
    ## Set color list
    if colors is None:
        if energies is None:
            color = np.zeros(Hs.shape[0])
        else:
            min_energy = energies.min()
            energies -= min_energy
            color = energies / (np.max(energies) + 1e-6)

    ## Grips
    grips = []
    for k in range(Hs.shape[0]):
        H = Hs[k, ...]

        if colors is None:
            c = color[k]
            c_vis = [0, 0, int(c * 254)]
        else:
            c_vis = list(np.array(colors[k, ...]))

        grips.append(
            create_gripper_marker(color=c_vis, scale=scale).apply_transform(H)
        )

    ## Visualize grips and the object
    if mesh is not None:
        scene = trimesh.Scene([mesh] + grips)
    elif c_cloud is not None:
        b = p_cloud.shape[0]
        colors = np.zeros((b, 3))
        colors[:, 1] = 255
        p_cloud_tri = trimesh.points.PointCloud(p_cloud, colors=colors)
        b = c_cloud.shape[0]
        colors = np.zeros((b, 3))
        colors[:, 0] = 255
        c_cloud_tri = trimesh.points.PointCloud(c_cloud, colors=colors)
        scene = trimesh.Scene([p_cloud_tri, c_cloud_tri] + grips)
    elif p_cloud is not None:
        b = p_cloud.shape[0]
        colors = np.zeros((b, 3))
        colors[:, 0] = 255 - ((p_cloud[:, 2] - p_cloud[:, 2].min()) / (p_cloud[:, 2].max() - p_cloud[:, 2].min()) * 255)
        colors[:, 1] = ((p_cloud[:, 2] - p_cloud[:, 2].min()) / (p_cloud[:, 2].max() - p_cloud[:, 2].min()) * 255)
        colors[:, 2] = 255
        p_cloud_tri = trimesh.points.PointCloud(p_cloud, colors=colors)
        scene = trimesh.Scene([p_cloud_tri] + grips)
    else:
        scene = trimesh.Scene(grips)

    if show:
        scene.show()
    else:
        return scene
