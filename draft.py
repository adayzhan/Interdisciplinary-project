import simpy

def func(env, action):
    while True:
        yield env.timeout(5)
        print('Stop')
        action.interrupt()
        yield env.timeout(20)

class A:
    def __init__(self, env):
        self.env = env
        self.action = env.process(self.run())

    def run(self):
        while True:
            try:
                # Start a new process for func
                self.proc = self.env.process(self.func())
                yield self.proc
                yield self.env.timeout(20)
            except simpy.Interrupt:
                print('Finish')
                self.proc.interrupt()
                # Explicitly terminate the process by stopping the loop
                break

    def func(self):
        try:
            for i in range(10):
                print(f'Run {i}')
                yield self.env.timeout(1)
        except simpy.Interrupt:
            pass

# Create SimPy environment
env = simpy.Environment()
a = A(env)
b = env.process(func(env, a.action))

# Run simulation
# env.run(20)

import numpy as np
# a = np.array([[1, 2], [3, 4]])
# b = np.array([[5, 6], [7, 8]])
# print(np.linalg.det(np.concatenate(((a[0] - a[1]).reshape(1, 2), (b[0] - b[1]).reshape(1, 2)), axis=0)))

def intersectInteriorRect(p, a: np.ndarray): # (2,) (4, 2)
    r = np.transpose((a-a[0])*(p-a[0])*(1, -1))
    print(r[1, 0])
    print(r[0] >= r[1])
a = np.array([[30, 20], [40, 20], [30, 10], [40, 10]])
p = np.array([15, 5])

r = []
for t in a:
    if np.all(t == a[0]):
        r.append([(p[0] - a[0][0])*(p[0] - a[0][0]), (p[1] - a[0][1])*(a[0][0]-p[0])])
    else:
        r.append([(t[0] - a[0][0])*(p[0] - a[0][0]), (t[1] - a[0][1])*(a[0][0]-p[0])])

x = 29
y = 6
w = 6
h = 6
m = 24
n = 12
t = -np.pi/4
sinT = np.sin(t)
cosT = np.cos(t)

p = np.array([m, n])
a = np.array([[x, y],
             [x - h * sinT, y + h * cosT],
             [x + w * cosT, y + w * sinT],
             [x + w * cosT - h * sinT, y + w * sinT + h * cosT]])

#intersectInteriorRect(p, a)
# (n, 2, 2) ((x, y), (i, j)) or ((x, y), (-1, -1))
import matplotlib.pyplot as plt

np.random.seed(0)
n = 4
a = np.random.uniform(0, 1, (n, 2))
b = np.random.uniform(0, 1, (n, 2))

fig, ax = plt.subplots(figsize=(3, 3))
ab_pairs = np.c_[a, b]
ab_args = ab_pairs.reshape(-1, 2, 2).swapaxes(1, 2).reshape(-1, 2)
print(ab_pairs)
print(ab_args)
print(ab_args.shape)
a = np.array([[[0, 1], [2, 3]], [[4, 5],  [6, 7]]])
print(a.reshape(-1, 2).transpose())
print(np.atan2(0, 0))
a = {1: 2, 3: 4}
