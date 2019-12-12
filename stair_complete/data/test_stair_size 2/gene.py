import numpy as np

list_size = np.array([0.864, 0.244, 0.126, 0.24])

total_arr = np.zeros((4, 20, 4))
total_arr[:, :, :] += list_size[None, None, :]

for i, item in enumerate(list_size):
    offset = np.linspace(1, -1, 20, endpoint=False)*item
    total_arr[i, :, i] += offset

total_arr = total_arr.reshape(-1, 4)
pass