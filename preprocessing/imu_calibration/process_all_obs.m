
clc;clear;
%% Load the data


base_path = '../../data/preprocessed_v1';

files = dir(base_path);

observations(numel(files)-2) = struct();

for i=1:numel(files)
    f = files(i);
    if ~f.isdir
        ld = load(fullfile(f.folder,f.name));
        fns = fieldnames(ld);
        for fn=1:numel(fns)
            observations(i-2).(fns{fn}) = ld.(fns{fn});
        end
    end
    X
end

%% Preprocess

rftFS = 1000;
imuFS = 100;

observations_processed = observations;

% for i=1:numel(observations_processed)
%     
%     obs = observations_processed(i);
%     
% 
% end

%% Interpolation

obs = observations_processed(100);

% Interpolation
tf = max(obs.rft1.time_steps(end), obs.rft2.time_steps(end));
tnom = 0:1/rftFS:tf;

method = 'pchip';
obs.rft1.force = interp1(obs.rft1.time_steps, obs.rft1.force, tnom, method);
obs.rft1.torque= interp1(obs.rft1.time_steps, obs.rft1.torque, tnom, method);
obs.rft1.time_steps = tnom;

% figure(1)
% subplot(2,1,1)
% plot(tnom,force_interp); grid on;
% subplot(2,1,2)
% plot(tnom,torque_interp); grid on;

figure(2)
subplot(2,1,1)
plot(obs.rft1.time_steps,obs.rft1.force); grid on;
subplot(2,1,2)
plot(obs.rft1.time_steps,obs.rft1.torque); grid on;

%% Low Pass filter for RFT

fx = obs.rft1.force(:,1);

%Filter design
Fs = 1000;
Hd = designfilt('lowpassfir','FilterOrder',100,'CutoffFrequency',12.5, ...
       'DesignMethod','window','Window',{@kaiser,3},'SampleRate',Fs);

%Plot magnitude of the filter
fvtool(Hd,1,'Fs',1000)

%Filter signal
fxtild = filter(Hd,fx);

%Plot frequency content of original signal vs frequency content of the
%filtered signal (this is in terms of omega, can also show it in hertz)
figure(5);
subplot(2,1,1);
x=linspace(0,2*pi,length(fx));
plot(x,abs(fft(fx))/max(abs(fft(fx))))
title('fft of original signal')

subplot(2,1,2);
plot(x,abs(fft(fxtild))/max(abs(fft(fxtild))))
title('fft of filtered signal')


%Plot signals in time domain
figure(5)
subplot(2,1,1);
plot(tnom,fx)
title('original signal')
grid on;

subplot(2,1,2);
plot(tnom,fxtild)
title('filtered signal')
grid on;

