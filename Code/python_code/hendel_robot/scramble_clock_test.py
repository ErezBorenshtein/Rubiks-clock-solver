import random

moveArr = [
    [0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],  # UR
    [0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0],  # DR
    [0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0],  # DL
    [1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # UL
    [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],  # U
    [0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0],  # R
    [0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],  # D
    [1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0],  # L
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],  # ALL
    [11, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0],  # UR
    [0, 0, 0, 0, 0, 0, 11, 0, 0, 0, 0, 1, 1, 1],  # DR
    [0, 0, 0, 0, 0, 0, 0, 0, 11, 0, 1, 1, 0, 1],  # DL
    [0, 0, 11, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0],  # UL
    [11, 0, 11, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0],  # U
    [11, 0, 0, 0, 0, 0, 11, 0, 0, 1, 0, 1, 1, 1],  # R
    [0, 0, 0, 0, 0, 0, 11, 0, 11, 0, 1, 1, 1, 1],  # D
    [0, 0, 11, 0, 0, 0, 0, 0, 11, 1, 1, 1, 0, 1],  # L
    [11, 0, 11, 0, 0, 0, 11, 0, 11, 1, 1, 1, 1, 1]  # ALL
]

invert = [-1, 1, -1, -1, -1, 5, -1, 7, -1, -1, -1, 11]

def select(n, k, idx):
    r = k
    val = 0
    for i in range(n - 1, -1, -1):
        if idx >= Cnk(i, r):
            idx -= Cnk(i, r)
            r -= 1
            val |= 1 << i
    return val

def randomState():
    return [random.randint(0, 11) for _ in range(14)]

def Solution(clock, solution):
    if len(clock) != 14 or len(solution) != 18:
        return -1
    return solveIn(14, clock, solution)

def swap(arr, row1, row2):
    arr[row1], arr[row2] = arr[row2], arr[row1]

def addTo(arr, row1, row2, startidx, mul):
    length = len(arr[0])
    for i in range(startidx, length):
        arr[row2][i] = (arr[row2][i] + arr[row1][i] * mul) % 12

ld_list = [7695, 42588, 47187, 85158, 86697, 156568, 181700, 209201, 231778]

def solveIn(k, numbers, solution):
    n = 18
    min_nz = k + 1

    for idx in range(Cnk(n,k)):
        val = select(n, k, idx)
        isLD = False
        for r in ld_list:
            if (val & r) == r:
                isLD = True
                break
        if isLD:
            continue
        map = [j for j in range(n) if (val >> j) & 1]
        arr = [[moveArr[map[j]][i] for j in range(k)] + [numbers[i]] for i in range(14)]
        ret = GaussianElimination(arr)
        if ret != 0:
            continue
        isSolved = all(arr[i][k] == 0 for i in range(k, 14))
        if not isSolved:
            continue
        backSubstitution(arr)
        cnt_nz = sum(1 for i in range(k) if arr[i][k] != 0)
        if cnt_nz < min_nz:
            solution[:] = [0] * 18
            for i in range(k):
                solution[map[i]] = arr[i][k]
            min_nz = cnt_nz
    return min_nz if min_nz != k + 1 else -1

def GaussianElimination(arr):
    m = 14
    n = len(arr[0])
    for i in range(n - 1):
        if invert[arr[i][i]] == -1:
            ivtidx = next((j for j in range(i + 1, m) if invert[arr[j][i]] != -1), -1)
            if ivtidx == -1:
                for j1 in range(i, m - 1):
                    for j2 in range(j1 + 1, m):
                        if invert[(arr[j1][i] + arr[j2][i]) % 12] != -1:
                            addTo(arr, j1, j2, i, 1)
                            ivtidx = j1
                            break
            if ivtidx == -1:
                if any(arr[j][i] != 0 for j in range(i + 1, m)):
                    return -1
                return i + 1
            swap(arr, i, ivtidx)
        inv = invert[arr[i][i]]
        for j in range(i, n):
            arr[i][j] = arr[i][j] * inv % 12
        for j in range(i + 1, m):
            addTo(arr, i, j, i, 12 - arr[j][i])
    return 0

def backSubstitution(arr):
    n = len(arr[0])
    for i in range(n - 2, 0, -1):
        for j in range(i - 1, -1, -1):
            if arr[j][i] != 0:
                addTo(arr, i, j, i, 12 - arr[j][i])

turns = ["UR", "DR", "DL", "UL", "U", "R", "D", "L", "ALL"]

def getScramble(type):
    rndarr = randomState()
    solution = [0] * 18
    Solution(rndarr, solution)
    scramble = ""

    for x in range(9):
        turn = solution[x]
        if turn == 0:
            continue
        clockwise = turn <= 6
        if turn > 6:
            turn = 12 - turn
        scramble += turns[x] + str(turn) + ("+" if clockwise else "-") + " "

    scramble += "y2 "

    for x in range(9):
        turn = solution[x + 9]
        if turn == 0:
            continue
        clockwise = turn <= 6
        if turn > 6:
            turn = 12 - turn
        scramble += turns[x] + str(turn) + ("+" if clockwise else "-") + " "

    isFirst = True
    for x in range(4):
        if random.randint(0, 1) == 1:
            scramble += (" " if not isFirst else "") + turns[x]
            isFirst = False

    return scramble

# Define Cnk function
def Cnk(n, k):
    """Binomial coefficient function."""
    if k < 0 or k > n:
        return 0
    k = min(k, n - k)
    c = 1
    for i in range(k):
        c *= n - i
        c //= i + 1
    return c

scrMgr = {'reg': lambda x, y: None}  # Placeholder for scrMgr.reg function, you can implement it as needed

clock = {'moveArr': moveArr}

print(getScramble(None))