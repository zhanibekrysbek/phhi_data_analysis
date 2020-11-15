function [obs,tf] = process_rft(obs)
%SMOOTH_POSE Summary of this function goes here
%   Detailed explanation goes here
    
    % Constants
    rftFS = 1000;
    cutoff = 12.5;
    method = 'pchip';
    % Low pass filter
    Hd = designfilt('lowpassfir','FilterOrder',100,'CutoffFrequency',cutoff, ...
           'DesignMethod','window','Window',{@kaiser,3},'SampleRate',rftFS);

    rft_ids = {'C00300119','C00300122'};

    
    tf = max(obs.rft1.time_steps(end), obs.rft2.time_steps(end));
    tnom = 0:1/rftFS:tf; % nominal time that will be common for both forces
    
    % Temporal alignement
    obs.rft1.force = interp1(obs.rft1.time_steps, obs.rft1.force, tnom, method);
    obs.rft1.torque= interp1(obs.rft1.time_steps, obs.rft1.torque, tnom, method);
    obs.rft1.time_steps = tnom;
    
    obs.rft2.force = interp1(obs.rft2.time_steps, obs.rft2.force, tnom, method);
    obs.rft2.torque= interp1(obs.rft2.time_steps, obs.rft2.torque, tnom, method);
    obs.rft2.time_steps = tnom;
    
    % Low pass filter
    obs.rft1.force = filter(Hd, obs.rft1.force);
    obs.rft1.torque = filter(Hd, obs.rft1.torque);
    
    obs.rft2.force = filter(Hd, obs.rft2.force);
    obs.rft2.torque = filter(Hd, obs.rft2.torque);
    
    % swap the sensors if variable names and sensor IDs are inconsistent
    if strcmp(obs.rft1.frame_id, rft_ids{2})
        temp = obs.rft1;
        obs.rft1 = obs.rft2;
        obs.rft2 = temp;
    end

end



