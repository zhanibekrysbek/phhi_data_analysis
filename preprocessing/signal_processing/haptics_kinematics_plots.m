

clc;clear;
base_path = '../../data/preprocessed_v2';

observations = load_data(base_path);


%%

obs = observations(3);
figure(1);
plot_rfts(obs,2)
% plot_pose(obs,1)
