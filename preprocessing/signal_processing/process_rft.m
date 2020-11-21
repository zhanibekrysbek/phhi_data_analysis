function [obs,tf] = process_rft(obs)
%PROCESS_RFT Summary of this function goes here
%   Detailed explanation goes here
    
    % Constants
    rftFS = 1000;
    cutoff = 12.5;
    method = 'pchip';
    % Low pass filter
    Hd = designfilt('lowpassfir','FilterOrder',100,'CutoffFrequency',cutoff, ...
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
    
    
    

    axangs = interp1(obs.pose123.time_steps, obs.pose123.orientation, obs.rft1.time_steps);

    rotm = axang2rotm(axangs);
    
    force_s = zeros(size(obs.rft1.force));
    force_s_1 = zeros(size(obs.rft1.force));
    torque_s = zeros(size(obs.rft1.torque));
    torque_s_1 = zeros(size(obs.rft1.torque));

    for i=1:length(axangs)

        force_s(i,:) = rotm(:,:,i)*obs.rft1.force(i,:)';
        force_s_1(i,:) = rotm(:,:,i)*obs.rft2.force(i,:)';

        torque_s(i,:) = rotm(:,:,i)*obs.rft1.torque(i,:)';
        torque_s_1(i,:) = rotm(:,:,i)*obs.rft2.torque(i,:)';

    end

    obs.rft1.forceS = force_s;
    obs.rft1.torqueS = torque_s;
    
    obs.rft2.forceS = force_s_1;
    obs.rft2.torqueS = torque_s_1;
    
    obs.fsum.force = obs.rft1.force + obs.rft2.force;
    obs.fsum.forceS = obs.rft1.forceS + obs.rft2.forceS;
    obs.fsum.time_steps = obs.rft1.time_steps;
    
    obs.fstretch.force = obs.rft1.force - obs.rft2.force;
    obs.fstretch.forceS = obs.rft1.forceS - obs.rft2.forceS;
    obs.fstretch.time_steps = obs.rft1.time_steps;

end



