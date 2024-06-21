current_dir = pwd;
rng(42);
policy_dir = fullfile(current_dir, 'policy');
data_dir = fullfile(current_dir, 'data');
figure_dir = fullfile(current_dir, 'figures');

if exist(policy_dir, 'dir') ~= 7
    mkdir(policy_dir);
end

if exist(data_dir, 'dir') ~= 7
    mkdir(data_dir);
end

if exist(figure_dir, 'dir') ~= 7
    mkdir(figure_dir);
end

addpath(genpath(fullfile(current_dir, 'libs')));

addpath(policy_dir);
addpath(data_dir);
addpath(figure_dir);

params_vehicles=[2,2,5,2]; %parameters of all vehicles: params_vehicles(id_vehicle,id_parameters)  [lr,lf,Vehicle_length,Vehicle_width]