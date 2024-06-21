clear;
clc;

%% Step 1: load parameters and add paths
addpath('config\');
param;

%% Step 2: load or generate the rnn-based target vehicle predictor
if exist(fullfile(policy_dir, 'policy.mat'), 'file') == 2
    retrain = false;
else
    retrain = true;
end
draw_figures = true;
[policy, RMSE_test] = train(retrain, draw_figures, policy_dir, figure_dir);

%% Step 3: solve the MPC for ego vehicle
if (exist(fullfile(data_dir, 'states_vehicles.mat'), 'file') == 2) && ...
        (exist(fullfile(data_dir, 'control_vehicles.mat'), 'file') == 2) && ...
            (exist(fullfile(data_dir, 'TV_prediction_all.mat'), 'file') == 2)
    load(fullfile(data_dir, 'states_vehicles.mat'), 'states_vehicles');
    load(fullfile(data_dir, 'control_vehicles.mat'), 'control_vehicles');
    load(fullfile(data_dir, 'TV_prediction_all'), 'TV_prediction_all');
else
    [states_vehicles, control_vehicles, TV_prediction_all] = solve(policy, params_vehicles, data_dir);
end

%% Step 4: draw the demonstration results
draw_trajectory(states_vehicles, figure_dir);
draw_profile(states_vehicles, control_vehicles, figure_dir);
draw_prediction(states_vehicles, TV_prediction_all, figure_dir);

%% Step 5: report collision instances
collision_records = check_collisions(states_vehicles, params_vehicles, data_dir);