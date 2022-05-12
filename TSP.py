import math
import numpy as np
import time
from random import sample
from math import inf

number = 5
max_distance = 20
distance = np.floor(np.random.random(number * number) * max_distance + 1)
# distance = np.zeros(number*number)+1

# distance = np.array([[math.inf, 10., 20., 18., 6.],
#                      [7., math.inf, 18., 8., 5.],
#                      [17., 8., math.inf, 13., 19.],
#                      [17., 20., 20., math.inf, 16.],
#                      [9., 1., 17., 7., math.inf], ])

# distance = np.array([[inf, 11., 17., 13., 9.],
#                      [19., inf, 9., 19., 4., ],
#                      [12., 20., inf, 19., 16.],
#                      [5., 15., 17., inf, 11.],
#                      [10., 19., 16., 14., inf], ])

distance.shape = (number, number)

for i in range(number):
    distance[i][i] = math.inf

print(distance)


def compute_dis(path):
    result = 0
    for i in path:
        result += distance[i]
    return result


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
        if not mins[i] == math.inf:
            for j in range(size[0]):
                mat[i][j] -= mins[i]
            bound += mins[i]

    for i in range(size[0]):
        distance_trans = mat.transpose()
        mins[size[0] + i] = min(distance_trans[i])
        if not mins[size[0] + i] == math.inf:
            for j in range(size[0]):
                mat[j][i] -= mins[size[0] + i]
            bound += mins[size[0] + i]

    return mat, bound


def test_complete(path):
    temp = np.zeros(number+1)
    for i in path:
        temp[i[0]+1] = i[1]+1

    next1 = path[0][0] +1
    visited = np.zeros(number+1)
    for i in range(number):
        next1 = temp[int(next1)]
        visited[int(next1)] += 1

    for i in range(1, len(visited)):
        if not visited[i] == 1:
            return False

    return True


def lc_search():
    best = math.inf
    best_path = []
    start = time.time()
    waiting_list = []
    ini_mat, ini_bound = trim(distance.copy(), 0)
    lc_search_procedure(ini_mat, ini_bound, waiting_list, [])
    while len(waiting_list) > 0:

        waiting_list.sort(key=(lambda x: len(x[2])), reverse=True)
        waiting_list.sort(key=(lambda x: x[0]))
        bound, mat, path = waiting_list[0]
        # print(len(waiting_list), best, path, mat)
        # print(waiting_list)
        waiting_list.pop(0)

        if len(path) >= number :
            if test_complete(path):
                result = compute_dis(path)
                if result < best:
                    best = result
                    best_path = path

        else :
            if mat.shape[0] + len(path) >= 5 and bound < best:
                lc_search_procedure(mat, bound, waiting_list, path)

    end = time.time()

    print("分枝定界: ", end - start, "s", sep='')
    print(best, best_path)


def lc_search_procedure(dis_mat, bound, waiting_list, path):
    zero_list = find_zero(dis_mat)
    np.random.shuffle(zero_list)
    for position in zero_list:
        path1 = path.copy()
        dis_mat_1 = dis_mat.copy()
        dis_mat_1[position] = math.inf
        dis_mat_1, bound1 = trim(dis_mat_1, bound.copy())
        waiting_list.append((bound1, dis_mat_1, path1))
        path2 = path.copy()
        path2.append(position)
        if len(path2) == number-1:
            path2.append((path2[-1][-1], path[0][0]))
        dis_mat_2 = dis_mat.copy()
        for i in range(number):
            dis_mat_2[i][position[1]] = math.inf
            dis_mat_2[position[0]][i] = math.inf
        dis_mat_2, bound2 = trim(dis_mat_2, bound.copy())
        waiting_list.append((bound2, dis_mat_2, path2))


def kruscal():
    start = time.time()
    result = math.inf
    best_path = []
    for i in range(number):
        k = 1
        dis_all = 0
        dis_mat = distance.copy()
        next1 = i
        path = []
        for j in range(number):
            dis_mat[j][i] = math.inf
        while k < number:
            k += 1
            next2 = np.argmin(dis_mat[next1][:])
            path.append((next1, next2))
            dis_all += dis_mat[next1][next2]
            for j in range(number):
                dis_mat[j][next2] = math.inf
            next1 = next2

        dis_all += distance[next1][i]
        path.append((next1, i))
        if dis_all < result:
            best_path = path
            result = dis_all

    end = time.time()
    print("Kruscal: ", end - start, "s", sep='')
    print(result, best_path)


def convert_node(random_proc):
    ini_path =[]
    for i in range(len(random_proc)-1):
        ini_path.append((random_proc[i],random_proc[i+1]))
    ini_path.append((random_proc[-1],random_proc[0]))
    return ini_path


def saa():
    start = time.time()
    random_proc = np.linspace(0,number-1,number,dtype=int)
    np.random.shuffle(random_proc)
    best_path = convert_node(random_proc)
    best = compute_dis(best_path)
    T = 10e20
    alpha = 0.99
    while T > 1e-20:
        if np.random.random()>0.5:
            random_int_list = sample(list(range(0, number)), 2)
            random_proc[random_int_list[0]], random_proc[random_int_list[1]], = \
            random_proc[random_int_list[1]],  random_proc[random_int_list[0]]
        else:
            random_int_list = sample(list(range(0, number)), 3)
            random_proc[random_int_list[0]], random_proc[random_int_list[1]], random_proc[random_int_list[2]] = random_proc[random_int_list[1]], random_proc[random_int_list[2]], random_proc[random_int_list[0]]

        path = convert_node(random_proc)
        dis = compute_dis(path)
        if dis < best or np.random.rand() < np.exp(-np.abs(best-dis)/T):
            best_path = path
            best = dis

        T *= alpha
    end = time.time()
    print("模拟退火: ", end - start, "s", sep='')
    print(best, best_path)


if __name__ == "__main__":
    saa()
    kruscal()
    lc_search()