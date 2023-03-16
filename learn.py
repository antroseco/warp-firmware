import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

jumping = pd.read_csv("jumping.csv")
running = pd.read_csv("running.csv")
walking = pd.read_csv("walking.csv")
still = pd.read_csv("stationary.csv")

print("Jumping length: ", len(jumping))
print("Running length: ", len(running))
print("Walking length: ", len(walking))
print("still length: ", len(still))


def make_vars(series, name):
    vars_x = []
    vars_y = []
    vars_z = []

    # FIXME
    for i in range(0, len(series), 32):
        vars_x.append(series["X"][i : i + 128].var())
        vars_y.append(series["Y"][i : i + 128].var())
        vars_z.append(series["Z"][i : i + 128].var())

    return list(zip(vars_x, vars_y, vars_z))


vars_walking = make_vars(walking, "walking")
vars_running = make_vars(running, "running")
vars_jumping = make_vars(jumping, "jumping")
vars_still = make_vars(still, "still")


def sigma(x):
    # Logistic function.
    return np.reciprocal(1 + np.exp(-x))


def log_likelihood_grad(w, Xs, Ys):
    # log-likelihood gradient (for gradient ascent).
    grad = np.zeros_like(w)

    for x, y in zip(Xs, Ys):
        grad += y * (1 - sigma(y * w @ x)) * x

    return grad


def gradient_ascent(Xs, Ys, iters):
    # Learn weights using gradient ascent.
    w = np.zeros_like(Xs[0])

    for _ in range(iters):
        w += 1e-2 * log_likelihood_grad(w, Xs, Ys)

    return w


def prepare(v):
    return np.hstack((1, v, v**2))


Xs_jumping = [prepare(np.log(np.asarray([x, y, z])) / 16) for x, y, z in vars_jumping]
Xs_running = [prepare(np.log(np.asarray([x, y, z])) / 16) for x, y, z in vars_running]
Xs_walking = [prepare(np.log(np.asarray([x, y, z])) / 16) for x, y, z in vars_walking]
Xs_still = [prepare(np.log(np.asarray([x, y, z])) / 16) for x, y, z in vars_still]


def learn_weights(Xs_target, Xs_2, Xs_3, Xs_4):
    # One big list.
    Xs = Xs_target + Xs_2 + Xs_3 + Xs_4
    Ys = [-1] * len(Xs_target) + [1] * (len(Xs_2) + len(Xs_3) + len(Xs_4))

    assert len(Xs) == len(Ys)

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    ax.scatter(
        [x[1] for x in Xs],
        [x[2] for x in Xs],
        [x[3] for x in Xs],
        s=20,
        c=Ys,
        cmap="RdYlBu",
    )

    plt.show()

    w = gradient_ascent(Xs, Ys, 400)

    predictions = [sigma(w @ x) for x in Xs]

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    ax.scatter(
        [x[1] for x in Xs],
        [x[2] for x in Xs],
        [x[3] for x in Xs],
        s=20,
        c=predictions,
        cmap="RdYlBu",
    )

    plt.show()

    print(w)


learn_weights(Xs_jumping, Xs_running, Xs_walking, Xs_still)
learn_weights(Xs_running, Xs_jumping, Xs_walking, Xs_still)
learn_weights(Xs_walking, Xs_jumping, Xs_running, Xs_still)
learn_weights(Xs_still, Xs_jumping, Xs_running, Xs_walking)
