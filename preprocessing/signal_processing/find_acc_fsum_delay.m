

clc;clear;
base_path = '../../data/preprocessed_v2_1';

[observations_processed,tb] = load_data(base_path);
observations_processed = adjust_time(observations_processed);


%% Find time offset

% for inds = 1:38: toff = 0
% for inds = 39:68: toff = 0.375
% for inds = 69:112: toff = 0.355

obs = observations_processed(112);
figure(1);
plot_phase(obs,1)



toff = 0.355;
obs.imu.time_steps = obs.imu.time_steps + toff;

figure(2);
plot_phase(obs,1)



