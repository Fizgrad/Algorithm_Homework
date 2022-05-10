import math
import numpy as np
import time

number = 5
max_distance = 20
distance = np.random.random(number*number)
distance.shape = (number, number)
distance = np.floor(distance*max_distance+1)
for i in range(number):
    distance[i][i] = math.inf

print(distance)


def find_zero(matrix):
    zero_list = []
    size = matrix.shape
    for i in range(size[0]):
        for j in range(size[1]):
            if matrix[i][j] == 0:
                zero_list.append((i, j))

    return zero_list


def trim(mat, bound):
    size = mat.shape
    mins = np.zeros(2 * size[0])
    for i in range(size[0]):
        mins[i] = min(mat[i])
        for j in range(size[0]):
            if not mins[i] == math.inf:
                mat[i][j] -= mins[i]

    for i in range(size[0]):
        distance_trans = mat.transpose()
        mins[size[0] + i] = min(distance_trans[i])
        for j in range(size[0]):
            if not mins[size[0] + i] == math.inf:
                mat[j][i] -= mins[size[0] + i]

    return mat, sum(mins) + bound


def lc_search():
    start = time.time()
    waiting_list = []
    ini_mat, ini_bound = trim(distance, 0)
    lc_search_procedure(ini_mat, ini_bound, waiting_list)
    while len(waiting_list) > 0:
        waiting_list.sort(key=(lambda x : x[0]))
        bound, mat = waiting_list[0]
        waiting_list.pop(0)
        if mat.shape ==(1, 1) and mat[0][0] ==0:
            end = time.time()
            print("分枝定界: ", end - start, "s", sep='')
            return bound
        if mat.shape ==(1, 1) and mat[0][0] ==math.inf:
            continue
        lc_search_procedure(mat, bound, waiting_list)



def lc_search_procedure(dis_mat, bound,waiting_list):
    zero_list = find_zero(dis_mat)
    for position in zero_list:
        dis_mat_1 = dis_mat.copy()
        dis_mat_1[position] = math.inf
        dis_mat_1, bound1 = trim(dis_mat_1, bound.copy())
        waiting_list.append((bound1, dis_mat_1))

        dis_mat_2 = dis_mat.copy()
        dis_mat_2 = np.delete(dis_mat_2, position[0], axis=0)
        dis_mat_2 = np.delete(dis_mat_2, position[1], axis=1)
        dis_mat_2, bound2 = trim(dis_mat_2, bound.copy())
        waiting_list.append((bound2, dis_mat_2))


if __name__ == "__main__":
    print(lc_search())
