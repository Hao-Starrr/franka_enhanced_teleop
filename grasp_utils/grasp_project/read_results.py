import numpy as np
import os
data = np.load('results.npz')

success = data['success']
fail = data['fail']
not_feasible = data['not_feasible']

print(np.sum(success, axis=0))
print(np.sum(fail, axis=0))
print(np.sum(not_feasible, axis=0))
print(np.sum(success), np.sum(fail))

PATH = '../../ycb_dataset_mesh/ycb/'
files = os.listdir(PATH)
for i in np.argwhere(np.sum(fail, axis=0) == 5)[:, 0]:
    print(files[i])
for i in np.argwhere(np.sum(not_feasible, axis=0) == 5)[:, 0]:
    print(files[i])


success = success[:, ~(np.sum(fail, axis=0) == 5)]
not_feasible = not_feasible[:, ~(np.sum(fail, axis=0) == 5)]
fail = fail[:, ~(np.sum(fail, axis=0) == 5)]

success = success[:, ~(np.sum(not_feasible, axis=0) == 5)]
fail = fail[:, ~(np.sum(not_feasible, axis=0) == 5)]
not_feasible = not_feasible[:, ~(np.sum(not_feasible, axis=0) == 5)]

print(np.sum(np.sum(success, axis=0) > np.sum(fail, axis=0)))
print(np.sum(np.sum(success, axis=0) <= np.sum(fail, axis=0)))
print(np.sum(success), np.sum(fail))

