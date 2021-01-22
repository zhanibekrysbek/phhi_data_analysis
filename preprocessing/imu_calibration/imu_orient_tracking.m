function observations_imu = imu_orient_tracking(observations_processed)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

observations_imu = adjust_time(observations_processed);

for i=progress(1:numel(observations_processed), 'Title', 'IMU Orient Tracking')
    observations_imu(i) = track_orientation(observations_imu(i));
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

    
    % 1 Gs, G = 0.0001 T
    accel = obs.imu.accel;
    gyro = obs.imu.gyro/180*pi;
    if strcmp(obs.obs_id, 'Sanket_Vignesh_1_1')
        gyro_mean = mean(gyro(end-29:end,:));
    else
        gyro_mean = mean(gyro(1:30,:));
    end
    
    gyro = gyro - gyro_mean;
    mag = obs.imu.mag; % convert from gauss to micro Tesla. Mag is pre calibrated

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
    
    obs = remove_gravity(obs);
    obs = spatial_angvel(obs);
    obs = compute_twist(obs);
    
end


function obs = compute_twist(obs)
twist = zeros([numel(obs.pose123.time_steps),6]);
twist(:,4:6) = obs.imu.angvelS;

for i = 1:numel(obs.pose123.time_steps)
    twist(i,1:3) = cross(obs.pose123.position(i,:)', obs.imu.angvelS(i,:)')' + obs.pose123.linvel(i,:);
end
obs.pose123.twist = twist;

end


function obs = remove_gravity(obs)

gS = [0;0;-9.81];

rotmats = axang2rotm(obs.imu.orientation);
gB = zeros(size(obs.imu.accel));
for i = 1:size(rotmats,3)
    gB(i,:) = gS'*rotmats(:,:,i);
end

obs.imu.pureAccel = obs.imu.accel - gB;

end


function obs = spatial_angvel(obs)

angvelS = zeros(size(obs.imu.angvel));  
rotmats = axang2rotm(obs.imu.orientation);

for i = 1:size(rotmats,3)
    angvelS(i,:) = obs.imu.angvel(i,:)*rotmats(:,:,i)';
end  

obs.imu.angvelS = angvelS;
    
end


