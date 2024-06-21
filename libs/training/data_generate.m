function [sample_item, label_item, index] = data_generate(lanes, horizons, t_stage, sampling_rate, num_velocities)

    source_lane = lanes(1);
    target_lane = lanes(2);
    history_len = horizons(1);
    predict_len = horizons(2);
    
    t = t_stage(1):sampling_rate:t_stage(4);
    n_stage_1 = floor(t_stage(2)/sampling_rate);
    n_stage_2 = floor(t_stage(3)/sampling_rate);
    n_stage_3 = length(t);
    
    index = 0;

    for v = 10:0.1:40

        x = t * v;
        y = ones(size(x))*source_lane;
        y(n_stage_1:n_stage_2) = data_interpolate(x(n_stage_1:n_stage_2) - x(n_stage_1), target_lane, source_lane);
        y(n_stage_2:end) = ones(size(y(n_stage_2:end)))*target_lane;
    
        for i = 0:n_stage_3 - history_len - predict_len
    
            index = index + 1;
            sample_x = x(i + 1:i + history_len) - x(i + history_len);
            sample_y = y(i + 1:i + history_len) - y(i + history_len);
            label_x = x(i + history_len + 1:i + history_len + predict_len) - x(i + history_len);
            label_y = y(i + history_len + 1:i + history_len + predict_len) - y(i + history_len);
            sample_item{index} = [sample_x; sample_y];
            label_item{index} = [label_x; label_y];
            
        end

    end