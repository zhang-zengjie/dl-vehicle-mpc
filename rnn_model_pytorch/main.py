import torch
from common import data_convert, data_generate
from torch.utils.data import DataLoader
from learning import trainer, validator, tester
from model import PredictionNet
import matplotlib.pyplot as plt


run_device = torch.device("cpu")
torch.manual_seed(42)

TRAINING_FLAG = True

batch_size = 30
learning_rate = 1e-2
num_epochs = 30
split_ratio = (0.6, 0.2, 0.2)

SAMPLING_RATE = 0.1
PREDICTION_TIME = 2
HISTORY_TIME = 3

T_STAGE_0 = 0
T_STAGE_1 = 2
T_STAGE_2 = 6
T_STAGE_3 = 8

Y_LANE_UP = 5.25 * 1.5
Y_LANE_DOWN = 5.25 * 0.5

history_len = round(HISTORY_TIME / SAMPLING_RATE)
predict_len = round(PREDICTION_TIME / SAMPLING_RATE)


def visualize_test(sample_traj, label_traj, prediction_traj):

    plt.plot(sample_traj[0], sample_traj[1], "bx", label="Past trajectory")
    plt.plot(label_traj[0], label_traj[1], "gx", label="Ground truth")
    plt.plot(prediction_traj[0], prediction_traj[1], 'r.')

    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x in m", fontsize=20)
    plt.ylabel("y in m", fontsize=20)
    # plt.xlim((-25, 55))
    # plt.ylim((-20, 30))
    plt.legend()
    plt.show()


if __name__ == "__main__":

    data_frame = data_generate(lanes=[Y_LANE_UP, Y_LANE_DOWN],
                               horizons=[history_len, predict_len],
                               t_stage=(T_STAGE_0, T_STAGE_1, T_STAGE_2, T_STAGE_3),
                               sampling_rate=SAMPLING_RATE,
                               num_velocities=150)
    train_frame, validation_frame, test_frame = data_convert(split=split_ratio, data_frame=data_frame)

    train_dataloader = DataLoader(train_frame, batch_size=batch_size, shuffle=True, drop_last=True)
    val_dataloader = DataLoader(validation_frame, batch_size=batch_size, shuffle=True, drop_last=True)

    if TRAINING_FLAG:
        net = PredictionNet(out_len=predict_len, batch_size=batch_size).to(run_device)
        trainer(net=net, lr=learning_rate, n_epochs=num_epochs, training_dataloader=train_dataloader,
                horizons=[history_len, predict_len])
        validator(net=net, dataloader=val_dataloader, horizons=[history_len, predict_len])
    else:
        net = torch.load('policy.pt')
    print('succeed')

    sample, label, prediction = tester(net, test_frame, horizons=[history_len, predict_len])
    num = 0
    visualize_test(sample[num], label[num], prediction[num])

