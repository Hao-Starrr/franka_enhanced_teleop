import numpy as np
import os

PATH = 'data/'
file_list = os.listdir(PATH)
print(file_list)

data_cleaned = []

for file in file_list:
    data = np.load(PATH + file)
    R = data['Rs'].reshape((-1, 3, 3))
    t = data['ts']
    col = data['cols']

    col_sign = col == 0
    R = R[col_sign]
    t = t[col_sign]

    t_sign = t[:, 2] > 0.1
    R = R[t_sign]
    t = t[t_sign]

    x = np.eye(4)[None, :, :].repeat(R.shape[0], axis=0)
    x[:, :3, :3] = R
    x[:, :3, 3] = t
    data_cleaned.append(x)

data_cleaned = np.concatenate(data_cleaned, axis=0)
print(data_cleaned.shape)
np.savez('data_merged.npz', data=data_cleaned)
