function pred = matpy(params_path, data)
    dlmodel = py.importlib.import_module('module');
    py.importlib.reload(dlmodel);
    model = py.importlib.import_module('dlprediction');
    py.importlib.reload(model);
    try
        pred = model.prediction(pyargs('params_path', params_path, 'data', data));
    catch
        pred = zeros(50, 4);
    end
    pred = double(pred);
end