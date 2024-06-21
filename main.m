param;

if exist(fullfile(policy_dir, 'policy.mat'), 'file') == 2
    retrain = false;
else
    retrain = true;
end

draw_figures = true;
[policy, RMSE_test] = train(policy_dir, retrain, draw_figures);

if (exist(fullfile(data_dir, 'states_vehicles.mat'), 'file') == 2) && ...
        (exist(fullfile(data_dir, 'control_vehicles.mat'), 'file') == 2) && ...
            (exist(fullfile(data_dir, 'TV_prediction_all.mat'), 'file') == 2)
    load(fullfile(data_dir, 'states_vehicles.mat'), 'states_vehicles');
    load(fullfile(data_dir, 'control_vehicles.mat'), 'control_vehicles');
    load(fullfile(data_dir, 'TV_prediction_all'), 'TV_prediction_all');
else
    [states_vehicles, control_vehicles, TV_prediction_all] = solve(policy, params_vehicles, data_dir);
end

draw_trajectory(states_vehicles, figure_dir);
draw_profile(states_vehicles, control_vehicles);
draw_prediction(states_vehicles, TV_prediction_all);

collision_records = check_collisions(states_vehicles, params_vehicles, data_dir);