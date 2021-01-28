function obs = get_fsum(obs)

    
    
    obs = get_force(obs);
    obs = get_torque(obs);
%     obs = rft_to_spatial(obs);
end


function obs = get_force(obs)
   % Fsum, Fstretch in Body and Spatial frames
    
    obs.fsum.force = obs.rft1.force + obs.rft2.force;

    obs.fsum.time_steps = obs.rft1.time_steps;
    
    obs.fstretch.force = obs.rft1.force - obs.rft2.force;
    
    obs.fstretch.time_steps = obs.rft1.time_steps;
end


function obs = get_torque(obs)
    
    obs.fsum.ttsum = obs.rft2.torque + obs.rft2.tcomp + obs.rft1.torque + obs.rft1.tcomp;

end

% Moved to imu_orient_tracking.m
function obs = rft_to_spatial(obs)

    axangs = interp1(obs.pose123.time_steps, obs.pose123.orientation, obs.rft1.time_steps);

    rotm = axang2rotm(axangs);
    
    force_s = zeros(size(obs.rft1.force));
    force_s_1 = zeros(size(obs.rft1.force));
    torque_s = zeros(size(obs.rft1.torque));
    torque_s_1 = zeros(size(obs.rft1.torque));
    ttsum_S = zeros(size(obs.rft1.torque));
    ttorque_1_S = zeros(size(obs.rft1.torque));
    ttorque_2_S = zeros(size(obs.rft1.torque));
    

    for i=1:length(axangs)

        force_s(i,:) = rotm(:,:,i)*obs.rft1.force(i,:)';
        force_s_1(i,:) = rotm(:,:,i)*obs.rft2.force(i,:)';

        torque_s(i,:) = rotm(:,:,i)*obs.rft1.torque(i,:)';
        torque_s_1(i,:) = rotm(:,:,i)*obs.rft2.torque(i,:)';
        
        ttorque_1_S(i,:) = rotm(:,:,i)*obs.rft1.ttorque(i,:)';
        ttorque_2_S(i,:) = rotm(:,:,i)*obs.rft2.ttorque(i,:)';
        
        ttsum_S(i,:) = rotm(:,:,i)*obs.fsum.ttsum(i,:)';

    end
    
    obs.rft1.forceS = force_s;
    obs.rft1.torqueS = torque_s;
    obs.rft1.ttorqueS = ttorque_1_S;
    
    obs.rft2.forceS = force_s_1;
    obs.rft2.torqueS = torque_s_1;
    obs.rft2.ttorqueS = ttorque_2_S;
    
    obs.fsum.forceS = obs.rft1.forceS + obs.rft2.forceS;
    obs.fstretch.forceS = obs.rft1.forceS - obs.rft2.forceS;
    
    obs.fsum.ttsum_S = ttsum_S;
    
end
