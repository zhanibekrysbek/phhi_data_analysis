function observations_imu = imu_orient_tracking(observations_processed)
%imu_orient_tracking Tracks the orientation from IMU data using Kalman
%Filter.

%   Also, with the same orientation data it converts the signals to spatial
%   frame.

observations_imu = adjust_time(observations_processed);

for i=progress(1:numel(observations_processed), 'Title', 'IMU Orient Tracking')
    obs = observations_imu(i);
    obs = track_orientation(obs);
    obs = rft2spatial(obs);
    obs = remove_gravity(obs);
    obs = imu2spatial(obs);
    obs = compute_twist(obs);
    observations_imu(i) = obs;
end

end


function obs = track_orientation(obs)

    % Identify Initial location (Table A or Table B)
    obs_id_num = split(obs.obs_id,'_');
    obs_id_num = str2double(obs_id_num{end});

    % grfA frame seen from ENU frame
    gA_ENU = [ 0 -1  0;
               1  0  0;
               0  0  1];
    % grfB frame seen from ENU frame
    gB_ENU = [ 0  1  0;
              -1  0  0;
               0  0  1];

    if mod(obs_id_num,2)==1
        gS_ENU = gA_ENU;
    elseif mod(obs_id_num,2)==0
        gS_ENU = gB_ENU;
    end

    
    accel = obs.imu.accel;
    gyro = obs.imu.gyro/180*pi;
    
    % Remove the bias from gyro, using the readings when sensor is at rest
    if strcmp(obs.obs_id, 'Sanket_Vignesh_1_1')
        gyro_mean = mean(gyro(end-29:end,:));
    else
        gyro_mean = mean(gyro(1:30,:));
    end
    
    gyro = gyro - gyro_mean;
    mag = obs.imu.mag; % convert from gauss to micro Tesla. Mag is pre calibrated

    % Convert it to ENU Frame
    if mag(1,2) < 0
        gS_ENU = gS_ENU';
    end
    
    
    filt = imufilter('ReferenceFrame','ENU',...
              'OrientationFormat','Rotation matrix');

    filt.SampleRate = 100;
    
    % Outputs the orientation of Tray wrt ENU
    [rotmats,angvel] = filt(accel, gyro);
    
    % Convert from ENU to grfA frame
    for i=1:size(rotmats,3)
        rotmats(:,:,i) = gS_ENU*rotmats(:,:,i);
    end

    % convert to axis-angle representation
    orient = rotm2axang(rotmats);
    orient = orient.*sign(orient(:,3));
    orient(:,4) = unwrap(orient(:,4));
    
    obs.imu.orientation = orient;
    obs.imu.angvel = angvel;
    
end


function obs = compute_twist(obs)

twistS = zeros([numel(obs.pose123.time_steps),6]);
twistS(:,4:6) = obs.imu.angvelS;

for i = 1:numel(obs.pose123.time_steps)
    twistS(i,1:3) = cross(obs.pose123.position(i,:)', obs.imu.angvelS(i,:)')' + obs.pose123.linvel(i,:);
    
end
obs.pose123.twistS = twistS;
obs.pose123.twistB = [obs.pose123.linvelB, obs.imu.angvel ];

end


function obs = remove_gravity(obs)

% gS = [0;0;-9.81];
gS = [0; 0; mean(obs.imu.accel(1:30,3))];

rotmats = axang2rotm(obs.imu.orientation);
gB = zeros(size(obs.imu.accel));
for i = 1:size(rotmats,3)
    gB(i,:) = gS'*rotmats(:,:,i);
end

obs.imu.pureAccel = obs.imu.accel - gB;

end

function obs = imu2spatial(obs)

    % Get Spatial IMU
    rotm = axang2rotm(obs.imu.orientation);
    
    accelS = zeros(size(obs.imu.accel));
    magS = zeros(size(obs.imu.mag));
    gyroS = zeros(size(obs.imu.gyro));
    angvelS = zeros(size(obs.imu.angvel));  
    pureAccelS = zeros(size(obs.imu.pureAccel)); 
    
    for i=1:length(accelS)
        accelS(i,:) = rotm(:,:,i)*obs.imu.accel(i,:)';
        magS(i,:) = rotm(:,:,i)*obs.imu.mag(i,:)';
        gyroS(i,:) = rotm(:,:,i)*obs.imu.gyro(i,:)';
        angvelS(i,:) = rotm(:,:,i)*obs.imu.angvel(i,:)';
        pureAccelS(i,:) = rotm(:,:,i)*obs.imu.pureAccel(i,:)';
    end
    
    obs.imu.accelS = accelS;
    obs.imu.magS = magS;
    obs.imu.gyroS = gyroS;
    obs.imu.angvelS = angvelS;
    obs.imu.pureAccelS = pureAccelS;

end

function obs = rft2spatial(obs)

    axangs = interp1(obs.imu.time_steps, obs.imu.orientation, obs.rft1.time_steps);

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

