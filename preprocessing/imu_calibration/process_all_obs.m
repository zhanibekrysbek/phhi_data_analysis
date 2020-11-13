

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

%% Low Pass filter


force = obs.rft1.force;
force = lowpass(force, 15, rftFS);


y = fft(force);

n = length(force);          % number of samples
f = (0:n-1)*(rftFS/n);     % frequency range
power = log10(abs(y).^2/n);    % power of the DFT

figure(3)
plot(f,power)
legend('x','y','z')
xlabel('Frequency')
ylabel('Power')
grid on;


figure(4)
subplot(2,1,1)
plot(tnom, force); grid on;
legend('x','y','z')
subplot(2,1,2)
plot(obs.rft1.time_steps,obs.rft1.torque); grid on;
legend('x','y','z')
