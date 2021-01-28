function [obs,tf] = process_rft(obs)
%PROCESS_RFT Summary of this function goes here
%   Detailed explanation goes here
    
    % Constants
    rftFS = 1000;
    lcutoff = 0;
    hcutoff = 12.5;
    method = 'pchip';

    rft_ids = {'C00300119','C00300122'};
    
    tf = min(obs.rft1.time_steps(end), obs.rft2.time_steps(end))-0.005;
    tf = round(tf,2); % ceil to second decimal place
    % nominal time that will be common for both forces
    tnom = 0:1/rftFS:tf; 
    
    % Temporal alignement
    obs.rft1.force = interp1(obs.rft1.time_steps, obs.rft1.force, tnom, method);
    obs.rft1.torque= interp1(obs.rft1.time_steps, obs.rft1.torque, tnom, method);
    obs.rft1.time_steps = tnom;
    obs.rft1.tnorm = tnom/tnom(end);
    
    obs.rft2.force = interp1(obs.rft2.time_steps, obs.rft2.force, tnom, method);
    obs.rft2.torque= interp1(obs.rft2.time_steps, obs.rft2.torque, tnom, method);
    obs.rft2.time_steps = tnom;
    obs.rft2.tnorm = tnom/tnom(end);
    
    % Low pass filter
    obs = lowpass_rft(obs, rftFS, lcutoff, hcutoff);
    
    
    % swap the sensors if variable names and sensor IDs are inconsistent
    if strcmp(obs.rft1.frame_id, rft_ids{2})
        temp = obs.rft1;
        obs.rft1 = obs.rft2;
        obs.rft2 = temp;
    end
    
    obs = process_torques(obs);
    
    obs.fsum = [];
    obs.fstretch = [];
    
end



function obs = lowpass_rft(obs,Fs, lcutoff, hcutoff)
    for ax = 1:3
        obs.rft1.force(:,ax) = bandpass_fft(obs.rft1.force(:,ax), Fs, lcutoff, hcutoff);
        obs.rft1.torque(:,ax) = bandpass_fft(obs.rft1.torque(:,ax), Fs, lcutoff, hcutoff);
        obs.rft2.force(:,ax) = bandpass_fft(obs.rft2.force(:,ax), Fs, lcutoff, hcutoff);
        obs.rft2.torque(:,ax) = bandpass_fft(obs.rft2.torque(:,ax), Fs, lcutoff, hcutoff);
    end
end


function obs = process_torques(obs)
    
rv1 = [ 0.235, 0, -0.027];
rv2 = [-0.235, 0, -0.027];

tcomp1 = cross(obs.rft1.torque, repmat(rv1,[numel(obs.rft1.time_steps), 1]));
tcomp2 = cross(obs.rft2.torque, repmat(rv2,[numel(obs.rft2.time_steps), 1]));

obs.rft1.tcomp = tcomp1;
obs.rft2.tcomp = tcomp2;
obs.rft1.ttorque = obs.rft1.torque+tcomp1;
obs.rft2.ttorque = obs.rft2.torque+tcomp2;

obs.fsum.ttsum = obs.rft2.torque+tcomp2 + obs.rft1.torque+tcomp1;

end



