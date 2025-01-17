import torch
import numpy as np
import onnxruntime as rt
from gpis import RGPIS
import copy


class GraspSampler:
    def __init__(self):
        self.sess = rt.InferenceSession("model.onnx")
        self.gp = RGPIS()

    def franka2kuka(self, H):
        H_new = np.eye(4)[np.newaxis, :, :].repeat(H.shape[0], 0)
        H_new[:, :3, 0] = H[:, :3, 1]
        H_new[:, :3, 1] = -H[:, :3, 0]
        H_new[:, :3, 2] = H[:, :3, 2]
        H_new[:, :3, 3] = H[:, :3, 3] - (0.15 - 1.12169998e-01) * H[:, :3, 2]
        return H_new

    def sampler(self, points):
        self.gp.train(points)
        points, normals = self.gp.get_surface_points(samples=16)

        input = np.concatenate(
            [points[np.newaxis, :, :, np.newaxis], normals[np.newaxis, :, :, np.newaxis]], axis=3)
        input = torch.from_numpy(input).to(torch.float).cpu().numpy()
        ort_inputs = {self.sess.get_inputs()[0].name: input}
        R, t, s, w = self.sess.run(None, ort_inputs)

        # is it a threhold grasp?
        mask = s[0] > 0.3  # 0.7
        R = R[0, mask]
        t = t[0, mask]
        w = w[0, mask]
        normals = normals[mask]

        n_grasps = R.shape[0]
        print("Number of grasps after threshold check: ", n_grasps)

        if n_grasps > 0:
            H = np.repeat(np.eye(4)[None, ...], n_grasps, axis=0)
            H[:, :3, :3] = R
            H[:, :3, 3] = t

            gripper_points = np.array([[4.10000000e-02, 0, 1.12169998e-01, 1],
                                       [-4.100000e-02, 0, 1.12169998e-01, 1],
                                       [0, 0, 6.59999996e-02, 1]])
            gripper_transform = np.swapaxes(
                H @ gripper_points.T, 1, 2)[:, :, :3]
            m, n, _ = gripper_transform.shape
            collision_check = np.sum(self.gp.occupancy_check(gripper_transform.reshape(-1, 3)).reshape(m, n, 1),
                                     axis=(1, 2)) == 0

            H = H[collision_check]
            w = w[collision_check]
            normals = normals[collision_check]
            n_grasps = H.shape[0]

        print("Number of grasps after collision check: ", n_grasps)

        if n_grasps > 0:
            c2 = 1.12169998e-01 * H[:, :3, 2] + \
                H[:, :3, 0] * w[:, np.newaxis] / 2
            c2_normal = self.gp.get_surface_normal(c2)
            normal_check = np.sum(normals * c2_normal, axis=1) < 0.0
            H = H[normal_check]
            w = w[normal_check]
            n_grasps = H.shape[0]

        print("Number of grasps after normal check: ", n_grasps)

        if n_grasps > 0:
            # Extract rotation matrices from H
            rotation_matrices = H[:, :3, :3]

            # Calculate rotation angles
            rotation_angles = np.array(
                [np.arccos(R[2, 2]) for R in rotation_matrices])

            # Now you have rotation angles for each grasp
            # You can perform further processing or checks based on these angles
            # For example, you can check if the rotation angle is within a certain range

            # Example: check if rotation angle is within 90 degrees of the vertical axis
            angle_threshold = np.radians(80)
            # angle_check = rotation_angles > 0
            vertical_angle_check = rotation_angles > angle_threshold
            # vertical_angle_check = angle_check & vertical_angle_check
            H = H[vertical_angle_check]
            w = w[vertical_angle_check]
            n_grasps = H.shape[0]

        print("Number of grasps after vertical angle check: ", n_grasps)

        if n_grasps > 0:
            H_reverse = copy.deepcopy(H)
            H_reverse[:, :3, 0] = -H[:, :3, 0]
            H_reverse[:, :3, 1] = -H[:, :3, 1]
            H = np.concatenate([H, H_reverse])
            # H = self.franka2kuka(H)
            w = np.concatenate([w, w])
            n_grasps = H.shape[0]

        print("Number of grasps :", n_grasps)

        if n_grasps > 0:
            return points, H, w
        return points, None, None
