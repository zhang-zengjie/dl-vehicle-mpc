import torch
from common import data_split

mse_loss = torch.nn.MSELoss()


def trainer(net, lr, n_epochs, training_dataloader, horizons, verbose=True):
    optimizer = torch.optim.Adam(params=list(net.parameters()), lr=lr)

    for j in range(n_epochs):
        if verbose:
            print("\nRunning epoch no. {:d}".format(j))
        net.train_flg = True

        for data in training_dataloader:

            sample, label = data_split(data, horizons)

            optimizer.zero_grad()
            loss = mse_loss(net(sample.float()), label.float())
            loss.backward()

            torch.nn.utils.clip_grad_norm_(net.parameters(), 10)
            optimizer.step()

        if verbose:
            print("Evaluating training epoch:")

    torch.save(net, 'policy.pt')


def validator(net, dataloader, horizons, verbose=True):

    net.train_flag = False
    acc_loss, num_samples = 0, 0

    for data in dataloader:
        sample, label = data_split(data, horizons)
        loss = mse_loss(net(sample.float()), label)

        acc_loss += loss.detach().cpu().numpy()
        num_samples += len(sample)

    if verbose:
        print("Validation loss is {:.2f} m^2 (Mean Squared Error)".format(acc_loss / num_samples))


def tester(net_in, test_data, horizons):
    sample, label = data_split(test_data, horizons)
    sample = sample.float()
    prediction = net_in(sample).detach().cpu().numpy()
    return sample, label, prediction
