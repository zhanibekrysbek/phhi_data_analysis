

%% Load the data

clc;clear;
base_path = '../../data/preprocessed_v1_1';

[observations, tb] = load_data(base_path);


%% Preprocess RFT and Pose

% Hd = designfilt('lowpassfir','FilterOrder',100,'CutoffFrequency',cutoff, ...
%        'DesignMethod','window','Window',{@kaiser,3},'SampleRate',rftFS);
   
rft_ids = {'C00300119','C00300122'};
observations_processed(numel(observations)) = struct('pose123',[], 'motion_type',[],...
            'obs_id',[], 'imu',[], 'traj_type', [], 'rft1',[], 'rft2',[], 'fsum',[], 'fstretch',[]);
profile on;
for i=progress(1:numel(observations_processed), 'Title','Preprocessing')
    
    [observations_processed(i),tf] = process_rft(observations(i));
     
    % ARUCO Pose Data
    % Removes Outliers,
    observations_processed(i) = process_pose(observations_processed(i),tf);
    
%     observations_processed(i) = obs;
end
% profile viewer;

%% IMU
obs = observations(1);

tf = min(obs.rft1.time_steps(end), obs.rft2.time_steps(end));
tf = round(tf - 0.005,2);
tnom = 0:1/100:tf; % nominal time that will be common for both forces

method = 'pchip';

% Temporal alignement
% fx = interp1(obs.imu.time_steps, obs.imu.accel(:,3), tnom, method);
fx = obs.rft1.force(:,3);
% obs.rft1.time_steps = tnom;



%fx = obs.imu.accel(:,3);

%Filter design
Fs = 1000;
% Hd = designfilt('lowpassfir','FilterOrder',61,'CutoffFrequency',15, ...
%        'DesignMethod','window','Window',{@kaiser,3},'SampleRate',Fs);

%Hd = fir1(61,0.3);

n = 101;
wo = 12.5/(Fs/2);
%wo = 0.3;
dp = 0.02;	
ds = 0.008;
Hd = fircls1(n,wo,dp,ds);

%Plot magnitude of the filter
% fvtool(Hd,1,'Fs',1000)
% fvtool(Hd,1)

%Filter signal
% fxtild = filter(Hd,1,fx-mean(fx));
fxtild = filter(Hd,1,fx);




% %Plot frequency content of original signal vs frequency content of the
% %filtered signal (this is in terms of omega, can also show it in hertz)
f = Fs/2*linspace(-1,1,length(fx));
figure(1)
subplot(2,1,1);
x=linspace(0,2*pi,length(fx));
plot(f,abs(fftshift(fft(fx-mean(fx)))))
title('fft of original signal')
grid on;

subplot(2,1,2);
plot(f,abs(fftshift(fft(fxtild))))
title('fft of filtered signal'); grid on;

% fxtild = fxtild+mean(fx);


%Plot signals in time domain
figure(2)
subplot(2,1,1);
plot(fx)
title('original signal')
grid on;

subplot(2,1,2);
plot(fxtild)
title('filtered signal')
grid on;



%Plot frequency content of original signal vs frequency content of the
%filtered signal (this is in terms of omega, can also show it in hertz)
% figure(1)
% subplot(2,1,1);
% x=linspace(0,2*pi,length(fx));
% plot(x,abs(fft(fx))/max(abs(fft(fx))))
% grid on;
% title('fft of original signal')
% 
% subplot(2,1,2);
% plot(x,abs(fft(fxtild))/max(abs(fft(fxtild))))
% grid on;
% title('fft of filtered signal')
% 
% 
% %Plot signals in time domain
% figure(2)
% subplot(2,1,1);
% plot(fx)
% grid on;
% title('original signal')
% 
% subplot(2,1,2);
% plot(fxtild)
% grid on;
% title('filtered signal')




% figure(3);
% subplot(211);
% plot(obs.imu.time_steps, obs.imu.accel); grid on;
% subplot(211)
% plot(obs.imu.time_steps, obs.imu.accel); grid on;



%% Save the data 1 by 1. GitHub has a strict limit to file size 100MB
base_path = '../../data/preprocessed_v2_1/';

for i=progress(1:numel(observations_processed), 'Title', 'Saving the data')
    obs = observations_processed(i);
    save([base_path obs.obs_id '.mat'], '-struct','obs');
end


%% Plot a dozen of samples
for i=progress(21:30)
    obs = observations_processed(i);
    obs2 = observations(i);
    
    figure(i+1);
    
    subplot(221)
    plot(obs.pose123.time_steps, obs.pose123.position,'.')
    % plot(obs.pose123.position,'.')
    grid on;
    subtitle('rmoutlier Position')
    ylabel('m')
%     legend('x','y','z')

    subplot(223)
    % plot(obs.pose123.orientation,'.')
    plot(obs.pose123.time_steps, obs.pose123.orientation,'.')

    grid on;
    subtitle('Orientation')
    xlabel('time, s')
    ylabel('axis-angle')
%     legend('x','y','z','angle')
    
    
    subplot(222)
    plot(obs2.pose123.time_steps, obs2.pose123.position,'.')
    % plot(obs.pose123.position,'.')
    grid on;
    subtitle('Raw Position')
    ylabel('m')
    legend('x','y','z')

    subplot(224)
    % plot(obs.pose123.orientation,'.')
    plot(obs2.pose123.time_steps, obs2.pose123.orientation,'.')

    grid on;
    subtitle('Orientation')
    xlabel('time, s')
    ylabel('axis-angle')
%     legend('x','y','z','angle')

    sgtitle(sprintf('%s %s %s Position', obs.obs_id, obs.traj_type, obs.motion_type ), ...
            'Interpreter','none');
    
end
