import time

import numpy as np

value = np.random.randint(1, 10, 20)
volume = np.random.randint(1, 10, len(value))
C = 2*len(value)

print("价值: ", value)
print("体积: ", volume)
print("背包容积: ", C)


def dp():
    start =time.time()
    result = np.zeros((len(value)+1, C+1))

    for j in range(1, C+1):
        for i in range(1, len(value)+1):
            result[i][j] = max(result[i][j-1], result[i-1][j])
            if volume[i-1] <= j:
                result[i][j] = max(result[i][j], result[i-1][j-volume[i-1]]+value[i-1])

    end = time.time()
    print("动态规划: ", end-start, "s", sep='')
    print(result[-1][-1])


def greedy():
    start = time.time()
    y = []
    for i in range(len(value)):
        y.append((value[i]/volume[i],i))
    y.sort(reverse=True)
    vacation = C
    result = 0
    for i in range(len(y)):
        if vacation >= volume[y[i][1]]:
            vacation = vacation - volume[y[i][1]]
            result = result + value[y[i][1]]
    end = time.time()
    print("贪心: ", end - start, "s", sep='')
    print(result)


def backtrace_entrance():
    start = time.time()
    result = [0]
    state = np.zeros(len(value))
    result_state = [1]
    depth = 0
    capacity = C
    backtrace_procedure(result, state, depth, capacity, result_state)
    end = time.time()
    print("回溯: ", end - start, "s", sep='')
    print(result[0])


def backtrace_procedure(result, state, depth, capacity, result_state):
    if depth >= len(value):
        value_all = np.dot(state, np.array(value).transpose())
        if result < value_all:
            result_state[0] = state.copy()
            result[0] = value_all

    else:
        if capacity >= volume[depth]:
            state[depth] = 1
            capacity -= volume[depth]
            backtrace_procedure(result, state, depth + 1, capacity, result_state)
            capacity += volume[depth]
            state[depth] = 0
        backtrace_procedure(result, state, depth + 1, capacity, result_state)


if __name__ == "__main__":
    dp()
    greedy()
    backtrace_entrance()


