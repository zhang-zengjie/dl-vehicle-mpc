import torch
import numpy as np
from module import PredictionNet
import torch._utils



def prediction(params_path, data):
    model = PredictionNet()
    model.load_state_dict(torch.load(params_path, map_location=torch.device('cpu')))
    model.eval()
    data = np.asarray(data)
    data = np.ascontiguousarray(data.T)
    data = torch.from_numpy(data)
    data = data.type(torch.FloatTensor)
    data = data.reshape(1, -1, data.shape[-2], data.shape[-1])

    with torch.no_grad():
        pred = model(data)
    pred = pred.detach().numpy().squeeze()
    pred = np.ascontiguousarray(pred.T)

    return pred