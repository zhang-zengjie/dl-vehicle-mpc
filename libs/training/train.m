function [policy, RMSE_test] = train(policy_dir, retrain, draw)

    rng(42);
    
    batch_size = 64;
    learning_rate = 1e-3;
    num_epochs = 30;
    split_ratio = [0.6, 0.2, 0.2];
    
    sampling_rate = 0.1;
    prediction_time = 3;
    history_time = 3;
    
    t_stage_0 = 0;
    t_stage_1 = 2;
    t_stage_2 = 6;
    t_stage_3 = 8;
    
    y_lane_up = 5.25 * 1.5;
    y_lane_down = 5.25 * 0.5;
    
    history_len = round(history_time / sampling_rate);
    predict_len = round(prediction_time / sampling_rate);
    
    num_velocities = 150;
    lanes = [y_lane_down, y_lane_up];
    horizons = [history_len, predict_len];
    t_stage = [t_stage_0, t_stage_1, t_stage_2, t_stage_3];
    
    [sample_frame, label_frame, data_len] = data_generate(lanes, horizons, t_stage, sampling_rate, num_velocities);
    shuffle_index = randperm(data_len);
    samples = sample_frame(shuffle_index);
    labels = label_frame(shuffle_index);
    
    Training_FLAG = 1;
    num_training = floor(split_ratio(1)*data_len);
    
    training_samples = samples(1:num_training);
    training_labels = labels(1:num_training);
    test_samples = samples(num_training + 1:end);
    test_labels = labels(num_training + 1:end);
    
    
    %% Model
    if Training_FLAG == retrain
    
        inputSize = 2;
        embeddingSize = 32;
        
        encoderRNNHiddenSize = 64;
        latentSize = 64;
        decoderRNNHiddenSize = 128;
        
        outputSize = 32;
        
        layers = [
            sequenceInputLayer(inputSize)
            layerNormalizationLayer
            gruLayer(encoderRNNHiddenSize)
            layerNormalizationLayer
            fullyConnectedLayer(latentSize)
            reluLayer
            gruLayer(decoderRNNHiddenSize)
            layerNormalizationLayer
            fullyConnectedLayer(inputSize)
            regressionLayer];
        
        %% Training
        
        options = trainingOptions("adam", ...
            MaxEpochs=num_epochs, ...
            SequencePaddingDirection="left", ...
            Shuffle="every-epoch", ...
            InitialLearnRate=learning_rate, ...
            MiniBatchSize=batch_size, ...
            ExecutionEnvironment='gpu', ...
            Verbose=true);
            
        [policy, info] = trainNetwork(training_samples(1:num_training), training_labels(1:num_training), layers, options);
    
        save(fullfile(policy_dir, 'policy.mat'), 'policy');
        save(fullfile(policy_dir, 'info.mat'), 'info');
    
    else
        load(fullfile(policy_dir, 'policy.mat'), 'policy');
        load(fullfile(policy_dir, 'info.mat'), 'info');
    end
    
    pred_labels = predict(policy, test_samples, SequencePaddingDirection="left");
    num = 310;
    
    RMSE_vec = calculate_cell_rmse(pred_labels, test_labels);
    RMSE_test = rmse(RMSE_vec, zeros(size(RMSE_vec)));
    
    if draw
        draw_training_rmse(info.TrainingRMSE)
        draw_test_rmse(RMSE_vec)
        draw_test_samples(test_samples{num}, test_labels{num}, pred_labels{num});
    end

end

