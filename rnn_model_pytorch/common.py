import numpy as np
import random
import torch


def data_interpolate(x, yf, y0):
    # f(x) = a_0 + a_1*x + a_2*(x^2) + a_3*(x^3);
    # Cubic spline to connect the strait lines.
    a_0 = y0
    a_1 = 0
    a_2 = 3 * (yf - y0) / (x[-1] ** 2)
    a_3 = -2 * (yf - y0) / (x[-1] ** 3)
    return a_0 + a_1 * x + a_2 * (x ** 2) + a_3 * (x ** 3)


def data_generate(lanes, horizons, t_stage, sampling_rate, num_velocities):

    source_lane = lanes[0]
    target_lane = lanes[1]
    history_len = horizons[0]
    predict_len = horizons[1]

    sample_length = history_len + predict_len
    sample_table = np.zeros((1, 2, sample_length))
    t = np.arange(t_stage[0], t_stage[3], sampling_rate)
    n_stage_1 = int(t_stage[1] / sampling_rate)
    n_stage_2 = int(t_stage[2] / sampling_rate)
    n_stage_3 = len(t)

    for v in np.linspace(10, 40, num_velocities):

        x = t * v
        y = np.full(x.shape, source_lane)
        y[n_stage_1:n_stage_2] = data_interpolate(x[n_stage_1:n_stage_2] - x[n_stage_1], target_lane, source_lane)
        y[n_stage_2:] = np.full(y[n_stage_2:].shape, target_lane)

        for i in range(n_stage_3 - sample_length):

            sample_x = x[i:i + sample_length] - x[i + history_len - 1]
            sample_y = y[i:i + sample_length]
            sample_item = np.vstack((sample_x, sample_y))
            sample_table = np.concatenate((sample_table, np.array([sample_item])), axis=0)

    sample_table = sample_table[1:, :, :]
    random.shuffle(sample_table)
    return sample_table


def data_split(data, horizons):
    return data[:, :, :horizons[0]], data[:, :, horizons[0]:]


def data_convert(split, data_frame):
    train_split, validation_split, test_split = split

    total_num_data = data_frame.shape[0]

    data_tensor = torch.tensor(data_frame)

    train_index = int(total_num_data * train_split)
    train_frame = data_tensor[:train_index]

    validation_index = int(total_num_data * validation_split) + train_index
    validation_frame = data_tensor[train_index:validation_index]

    test_frame = data_tensor[validation_index:]

    return train_frame, validation_frame, test_frame
