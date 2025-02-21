import random

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
print("Still length: ", len(still))


def make_vars(series):
    vars_max = []
    vars_mid = []
    vars_min = []

    # FIXME
    for i in range(0, len(series) - 32, 16):
        vars = [
            series["X"][i : i + 32].var(),
            series["Y"][i : i + 32].var(),
            series["Z"][i : i + 32].var(),
        ]

        vars_max.append(np.max(vars))
        vars_mid.append(np.median(vars))
        vars_min.append(np.min(vars))

    return list(zip(vars_max, vars_mid, vars_min))


vars_walking = make_vars(walking)
vars_running = make_vars(running)
vars_jumping = make_vars(jumping)
vars_still = make_vars(still)


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


# Take the log and divide by 16 to normalize values to the [0, 1] range.
Xs_jumping = [prepare(np.log(np.asarray([x, y, z])) / 16) for x, y, z in vars_jumping]
Xs_running = [prepare(np.log(np.asarray([x, y, z])) / 16) for x, y, z in vars_running]
Xs_walking = [prepare(np.log(np.asarray([x, y, z])) / 16) for x, y, z in vars_walking]
Xs_still = [prepare(np.log(np.asarray([x, y, z])) / 16) for x, y, z in vars_still]


def split_test_train(Xs):
    random.shuffle(Xs)
    return Xs[:-8], Xs[-8:]


# Quick-and-dirty split into training and testing data.
Xs_train_jumping, Xs_test_jumping = split_test_train(Xs_jumping)
Xs_train_running, Xs_test_running = split_test_train(Xs_running)
Xs_train_walking, Xs_test_walking = split_test_train(Xs_walking)
Xs_train_still, Xs_test_still = split_test_train(Xs_still)


def plot_3d(Xs, Ys, name):
    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    ax.scatter(
        [x[1] for x in Xs],
        [x[2] for x in Xs],
        [x[3] for x in Xs],
        s=20,  # type: ignore
        c=Ys,
        cmap="RdYlBu",
    )

    ax.set_title(name)

    ax.set_xlabel("Max")
    ax.set_ylabel("Mid")
    ax.set_zlabel("Min")  # type: ignore

    fig.tight_layout()

    plt.show()


def learn_weights(Xs_target, Xs_2, Xs_3, Xs_4, name):
    # One big list.
    Xs = Xs_target + Xs_2 + Xs_3 + Xs_4
    Ys = [1] * len(Xs_target) + [-1] * (len(Xs_2) + len(Xs_3) + len(Xs_4))

    # plot_3d(Xs, Ys, name)

    assert len(Xs) == len(Ys)

    w = gradient_ascent(Xs, Ys, 10_000)

    predictions = [sigma(w @ x) for x in Xs]
    plot_3d(Xs, predictions, name)

    return w


w_jumping = learn_weights(
    Xs_train_jumping, Xs_train_running, Xs_train_walking, Xs_train_still, "Jumping"
)
print(w_jumping)

w_running = learn_weights(
    Xs_train_running, Xs_train_jumping, Xs_train_walking, Xs_train_still, "Running"
)
print(w_running)

w_walking = learn_weights(
    Xs_train_walking, Xs_train_jumping, Xs_train_running, Xs_train_still, "Walking"
)
print(w_walking)

w_still = learn_weights(
    Xs_train_still, Xs_train_jumping, Xs_train_running, Xs_train_walking, "Still"
)
print(w_still)

# Resulting weights:
# [-13.10811183   4.918834    -2.17001111  -0.49517191  23.71468085 -7.62446347  -7.41696791]
# [-24.9530344   -1.9296893   12.29307804   6.52435167 -19.71056821 25.42783071  15.08661804]
# [-59.07509115  48.33650435  63.63207188  64.42407662 -43.04040799 -37.79053592 -50.70542156]
# [ 19.53366281   4.17987552  -6.32959318  -7.20169904  -2.07712563 -15.868433   -14.58375232]


def predict_softmax(x, w_desired, *w_others):
    def evaluate(w):
        return np.exp(w @ x)

    e_desired = evaluate(w_desired)
    e_others = sum(map(evaluate, w_others))

    return e_desired / (e_desired + e_others)


def evaluate_test_data(Xs_test, w_test, *w_others):
    predicted = []
    for x_test in Xs_test:
        predicted.append(predict_softmax(x_test, w_test, *w_others))

    return np.mean(predicted)


print(evaluate_test_data(Xs_test_jumping, w_jumping, w_running, w_walking, w_still))
print(evaluate_test_data(Xs_test_running, w_running, w_jumping, w_walking, w_still))
print(evaluate_test_data(Xs_test_walking, w_walking, w_running, w_jumping, w_still))
print(evaluate_test_data(Xs_test_still, w_still, w_running, w_walking, w_jumping))

# Resulting scores:
# 0.91546253293149
# 0.8644648849944065
# 0.6853301855246644
# 0.9418338548353613
