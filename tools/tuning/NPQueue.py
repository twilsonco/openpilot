#!/opt/homebrew/bin/python3

import timeit
import numpy as np

class NPQueue:
    def __init__(self, maxlen, rowsize):
        self.maxlen = maxlen
        self.rowsize = rowsize
        self.arr = np.empty((maxlen, rowsize))
        self.start = 0
        self.length = 0

    def __len__(self):
        return self.length

    def append(self, pt):
        if self.length < self.maxlen:
            self.arr[self.length] = pt
            self.length += 1
        else:
            self.arr[self.start] = pt
            self.start = (self.start + 1) % self.maxlen

    def append_old(self, pt):
        if len(self.arr) < self.maxlen:
            self.arr = np.append(self.arr, [pt], axis=0)
        else:
            self.arr[:-1] = self.arr[1:]
            self.arr[-1] = pt

# Create an instance of NPQueue with maxlen=5 and rowsize=3
queue = NPQueue(5, 3)

# Benchmark the two append methods
n = 1000000
pt = [1, 2, 3]
t1 = timeit.timeit(lambda: queue.append(pt), number=n)
t2 = timeit.timeit(lambda: queue.append_old(pt), number=n)

# Print the results
print(f"append: {t1:.6f} seconds")
print(f"append_old: {t2:.6f} seconds")