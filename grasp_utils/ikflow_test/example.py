from ikflow.utils import set_seed
from ikflow.model_loading import get_ik_solver

import numpy as np
import matplotlib.pyplot as plt

set_seed()
ik_solver, hyper_parameters = get_ik_solver("iiwa7_full_temp_nsc_tpm")
robot = ik_solver.robot

import timeit

parameter = []
measure = []

for i in [50]: #range(1, 51):
    n_ee = i * 2
    repeats = 1
    target_poses = np.array(
        [
            [0.25, 0, 0.5, 1, 0, 0, 0],
            [0.35, 0, 0.5, 1, 0, 0, 0],
            [0.45, 0, 0.5, 1, 0, 0, 0],
            [0.55, 0, 0.5, 1, 0, 0, 0],
            [0.65, 0, 0.5, 1, 0, 0, 0],
        ] * n_ee
    )

    solutions, l2_errors, angular_errors, joint_limits_exceeded, self_colliding, runtime =ik_solver.solve_n_poses(target_poses, refine_solutions=False, return_detailed=True)
    print(l2_errors)
    print(angular_errors)
    # def function():
    #     return ik_solver.solve_n_poses(target_poses, refine_solutions=False, return_detailed=True)
    # parameter.append(n_ee)
    # measure.append(timeit.timeit(function, number=repeats) / repeats)

# plt.xlabel('Number of target ee poses')
# plt.ylabel('Average execution time (sec)')
# plt.xlim(parameter[0], parameter[-1])
# plt.plot(parameter, measure)
# plt.title("IKFlow test (with refinement)")
# plt.grid(True)
# plt.show()
