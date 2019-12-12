# Temporarly Abandoned because of Module Path

import numpy as np
from sklearn.neighbors import NearestNeighbors


def knn_filter(samples, k=5, dist_threshold = 2):
    neigh = NearestNeighbors(k)
    neigh.fit(samples)
    dist, ind = neigh.kneighbors(return_distance=True)

    value_table = dist > dist_threshold
    index = np.nonzero(value_table)

    filtered_samples = np.delete(samples, index[0], axis=0)

    return filtered_samples


samples = [[0, 0, 2], [1, 0, 0], [0, 0, 1], [2, 1, 0]]

samples = np.array(samples)

print(knn_filter(samples, 2))