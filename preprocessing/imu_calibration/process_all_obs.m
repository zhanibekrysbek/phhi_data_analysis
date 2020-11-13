

%% Load the data
base_path = '/home/zhanibek/codes/phhi_data_analysis/data/preprocessed_v1';
clear observations;

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

obs = observations_processed(1);

% Interpolation
rftFS = 1000;
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
% force = lowpass(force, 15, rftFS);


nfft = length(fx);          % number of samples
nfft2 = 2.^nextpow2(nfft);
y = fft(fx, nfft2);
y = y(1:nfft2/2);

xfft = rftFS.*(0:nfft2/2-1)/nfft2;


figure(3)
plot(xfft,abs(y/max(y)));
% legend('x','y','z')
xlabel('Frequency')
ylabel('Power')
grid on;


cutoff = 10/rftFS/2;
h = fir1(32,cutoff);
fx_filt = conv(fx,h);

figure(4)
subplot(211)
plot(tnom, fx);
subplot(212)
plot(fx_filt);

% figure(4)
% subplot(2,1,1)
% plot(tnom, force); grid on;
% legend('x','y','z')
% subplot(2,1,2)
% plot(obs.rft1.time_steps,obs.rft1.torque); grid on;
% legend('x','y','z')



%% 

dp=0.01;
ds=0.01;
fp=0.36-595*10^-4;
fs=0.44+595*10^-4;
Fsampl=2;

[nbut_ord, wn] = buttord(2*fp/Fsampl, 2*fs/Fsampl,-20*log10(1-dp),-20*log10(ds));
[b_but,a_but] = butter(nbut_ord, wn);
Hbut=freqz(b_but,a_but,501);
figure
plot(abs(Hbut))
title('Butterworth lowpass filter magnitude response')
xlabel('frequency')
ylabel('magnitude')
nbut_ord

