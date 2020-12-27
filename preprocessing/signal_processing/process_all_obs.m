

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
    
    observations_processed(i) = process_imu(observations_processed(i),tf);
    
%     observations_processed(i) = obs;
end
% profile viewer;

%% IMU Preprocessing
obs = observations_processed(1);

% Temporal alignement
% fx = interp1(obs.imu.time_steps, obs.imu.accel(:,3), tnom, method);
fx = obs.imu.accel(:,1);
% obs.rft1.time_steps = tnom;



%fx = obs.imu.accel(:,3);

%Filter design
Fs = 100;
% Hd = designfilt('lowpassfir','FilterOrder',61,'CutoffFrequency',15, ...
%        'DesignMethod','window','Window',{@kaiser,3},'SampleRate',Fs);

%Hd = fir1(61,0.3);

n = 101;
wo = 20/(Fs/2);
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


%% Low pass filtering using fft and ifft

obs = observations(112);
x = obs.imu.accel(:,3);
hcutoff = 12.5;
lcutoff = 0.05;

Fs = 100;

y = fft(x);     
f = (0:length(y)-1)*Fs/length(y);

n = length(x);                         
fshift = (-n/2:n/2-1)*(Fs/n);
yshift = fftshift(y);
I = (fshift<=hcutoff & fshift>=lcutoff) | (fshift>=-hcutoff & fshift<=-lcutoff);

yshift(~I) = 0;

figure(1);
plot(fshift,abs(yshift))
title('Magnitude')

ynew = ifftshift(yshift);
xnew = ifft(ynew);



figure(2);
subplot(2,1,1);
plot(x);%hold on;
grid on;
% plot(real(xnew)); hold off
yl = ylim;
subplot(2,1,2);
plot(real(xnew));hold off;
grid on;
ylim(yl)

legend('original', 'filtered')



%% Save the data 1 by 1. GitHub has a strict limit to file size 100MB
base_path = '../../data/preprocessed_v2_1/';

for i=progress(1:numel(observations_processed), 'Title', 'Saving the data')
    obs = observations_processed(i);
    save([base_path obs.obs_id '.mat'], '-struct','obs');
end


%% Plot a dozen of samples
for i=progress(21:25)
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




%% Comparison of Force signals before and after low pass filter

ind = 43;
obs1 = observations(ind);
[obs2,~] = process_rft(obs1);

rft_ids = {'C00300119','C00300122'};
% swap the sensors if variable names and sensor IDs are inconsistent
if strcmp(obs1.rft1.frame_id, rft_ids{2})
    temp = obs1.rft1;
    obs1.rft1 = obs1.rft2;
    obs1.rft2 = temp;
end


figure(1);

subplot(3,1,1);
plot(obs1.rft1.time_steps, obs1.rft1.force(:,1)); hold on;
plot(obs2.rft1.time_steps, obs2.rft1.force(:,1)); hold off;
subtitle('x axis')

subplot(3,1,2);
plot(obs1.rft1.time_steps, obs1.rft1.force(:,2)); hold on;
plot(obs2.rft1.time_steps, obs2.rft1.force(:,2)); hold off;
subtitle('y axis')

subplot(3,1,3);
f1 = plot(obs1.rft1.time_steps, obs1.rft1.force(:,3)); hold on;
f2 = plot(obs2.rft1.time_steps, obs2.rft1.force(:,3)); hold off;
subtitle('z axis')

legend([f1,f2], {'before', 'after'});



%% Comparison of RFT and Acceleration prior to filtering

ind = 60;
obs = observations(ind);
[obs2,~] = process_rft(obs1);

rft_ids = {'C00300119','C00300122'};
% swap the sensors if variable names and sensor IDs are inconsistent
if strcmp(obs.rft1.frame_id, rft_ids{2})
    temp = obs.rft1;
    obs.rft1 = obs.rft2;
    obs.rft2 = temp;
end

% Constants
rftFS = 1000;
hcutoff = 12.5;
method = 'pchip';
% Low pass filter
Hd = designfilt('lowpassfir','FilterOrder',100,'CutoffFrequency',hcutoff, ...
       'DesignMethod','window','Window',{@kaiser,3},'SampleRate',rftFS);

rft_ids = {'C00300119','C00300122'};

tf = min(obs.rft1.time_steps(end), obs.rft2.time_steps(end))-0.005;
tf = round(tf,2); % ceil to second decimal place
% nominal time that will be common for both forces
tnom = 0:1/rftFS:tf; 

% Temporal alignement
obs.rft1.force = interp1(obs.rft1.time_steps, obs.rft1.force, tnom, method);
obs.rft1.torque= interp1(obs.rft1.time_steps, obs.rft1.torque, tnom, method);
obs.rft1.time_steps = tnom;

obs.rft2.force = interp1(obs.rft2.time_steps, obs.rft2.force, tnom, method);
obs.rft2.torque= interp1(obs.rft2.time_steps, obs.rft2.torque, tnom, method);
obs.rft2.time_steps = tnom;

    
    
    
    
    
fsum = obs.rft1.force + obs.rft2.force;
obs.fsum.force = fsum;   
obs.fsum.time_steps = obs.rft1.time_steps;
figure(20);

    subplot(3,1,1);
    plot(obs.rft1.time_steps, fsum(:,1)); hold on;
    plot(obs.imu.time_steps, obs.imu.accel(:,1)); hold off;
    subtitle('x - axis')
    grid on;

    subplot(3,1,2);
    plot(obs.rft1.time_steps, fsum(:,2)); hold on;
    plot(obs.imu.time_steps, obs.imu.accel(:,2)); hold off;
    subtitle('y - axis')
    grid on;

    subplot(3,1,3);
    f1 = plot(obs.rft1.time_steps, fsum(:,3)); hold on;
    a1 = plot(obs.imu.time_steps, obs.imu.accel(:,3)); hold off;
    subtitle('z - axis')
    grid on;

    legend([f1,a1], {'force', 'accel'});



figure(21);
    subplot(2,1,1)
    % Phase Angles
    fsum_ph = atan2d(obs.fsum.force(:,2),obs.fsum.force(:,1));
%     fsum_ph = unwrap(fsum_ph,180);

    imuacc_ph = atan2d(obs.imu.accel(:,2), obs.imu.accel(:,1));
%     imuacc_ph = unwrap(imuacc_ph,180);

    fs = plot(obs.rft1.time_steps, fsum_ph,'.'); hold on;
    iacc = plot(obs.imu.time_steps, imuacc_ph,'.'); hold off;

    hold off; grid on;
    ylabel('vector angles, deg');
    xts = xticks;
    xticks(xts(1):xts(end));

    subplot(2,1,2)
    % Signal Magnitude
    fsm = plot(obs.fsum.time_steps, vecnorm(obs.fsum.force(:,1:2),2,2)); hold on;
    iaccm = plot(obs.imu.time_steps, vecnorm(obs.imu.accel(:,1:2),2,2)); hold off;
    grid on;


