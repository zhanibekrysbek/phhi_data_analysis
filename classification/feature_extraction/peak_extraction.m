
clc;clear;
base_path = '../../data/preprocessed_v2_2';

[observations_processed,tb] = load_data(base_path);

%% Extract necessary information from the observation

% select observation
obs_ind = 74;
obs = observations_processed(obs_ind);

% original time and force data 
t_org = obs.fstretch.time_steps(1:10:end);
fx_org = obs.fstretch.force(1:10:end,1);
fz = obs.fsum.force(1:10:end,3);

% t0 and td index
td_idx = find(t_org>=obs.tdec_sec, 1);
t0_idx = find(fz>=0.8*mean(fz), 1);

% filtered time and force btw t0~td
fx_fltrd = fx_org(t0_idx:td_idx,:);
t_fltrd = t_org(t0_idx:td_idx);

%% Design radial basis network
% parameters
mseg = 0.01; % mse goal
sc = 0.5;   % spread constant
mn = round(200*(t_fltrd(end)-t_fltrd(1)));  % maximum number of neurons
df = round(mn/10);   % # of neurons to add btw displays

% train network
net = newrb(t_fltrd, fx_fltrd', mseg, sc, mn, df);
disp("Training has finished.");
% test set
app_fx = net(t_fltrd);

%% Find peaks and its windows

% absolute function of the approximation
abs_app_fx = abs(app_fx); 
% total number of samples
num_samples = length(app_fx);

% find local maximum 
[~,max_locs] = findpeaks(abs_app_fx,'MinPeakWidth',round(num_samples/15));
local_max = app_fx(max_locs);
local_max_time = t_fltrd(max_locs);

% find local min
abs_app_fx_inv = 1.01*max(abs_app_fx)-abs_app_fx;
[~,min_locs] = findpeaks(abs_app_fx_inv,'MinPeakWidth',round(num_samples/30));
% included the first and final sample
min_locs = [1 min_locs num_samples];
local_min = app_fx(min_locs);
local_min_time = t_fltrd(min_locs);

% plot the original and approximated function
figure(1);
plot(t_fltrd, fx_fltrd, t_fltrd, app_fx);
grid on; box on; hold on;
plot(local_max_time,local_max,'o','MarkerSize',8);
xlabel("Time");
ylabel("N");
subtitle("Fstretch Compare");
y_max = max(app_fx)+5;
y_min = min(app_fx)-5;
ylim([y_min y_max]);

% finds the windows of the peaks and plot vertical lines
win_locs = [];
min_time1 = local_min_time(1);
max_idx = 1;
min_idx = 2;
while max_idx <= length(local_max_time)
    pk_time = local_max_time(max_idx);
    min_time2 = local_min_time(min_idx);
    if pk_time > min_time1 && pk_time < min_time2
        win_locs = [win_locs; min_time1 min_time2];
        xline(min_time1,'-.b', 't0');
        xline(min_time2,'-.b', 'tf','LabelVerticalAlignment','bottom');
        min_time1 = min_time2;
        temp = min_idx;
        max_idx = max_idx + 1;
    end
    if min_idx == length(local_min_time)
        min_idx = temp + 1;
        max_idx = max_idx + 1;
    else
        min_idx = min_idx + 1;
    end
end
hold off;



figure(2);

plot_rfts(obs,1);