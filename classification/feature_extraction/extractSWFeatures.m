function [X,Y] = extractSWFeatures(observations_processed,option)
%extractSWFeatures Summary of this function goes here
%   Output: 
%       X - data matrix: 
%           rows - observations, columns - features
%       Y - label: 
%           Y = [obs_id starting_table window_num traj_type motion_type]
%           



global wind_size stride divisions opt numHapticSignals numSignals;

wind_size = 0.03;
stride = 0.01;
divisions = 5;

opt = option; % Option for feature data
opt2numsig = [28, 28, 58, 58, 30]; % number of signals

numSignals = opt2numsig(opt); % number of signals
numHapticSignals = 30; % number of Haptic signals

if opt >= 3
   hfeatures = haptic_features(observations_processed);
end

Nwinds = ceil((1 - wind_size)/stride);

X = zeros(Nwinds*numel(observations_processed), divisions*numSignals);
Y = zeros(Nwinds*numel(observations_processed), 5);

for ind=progress(1:numel(observations_processed), 'Title', 'ExtractingSWFeatures')
    
    obs = observations_processed(ind);
    if opt >= 3
       obs.haptics = hfeatures(ind).haptics;
    end
    
    [X((ind-1)*Nwinds+1:ind*Nwinds,:), Y((ind-1)*Nwinds+1:ind*Nwinds,3:end)] = ...
        sliding_window(obs);
      
    Y((ind-1)*Nwinds+1:ind*Nwinds,1) = ind; % obs_id
    
    % Identify Initial location (Table A or Table B)
    obs_id_num = split(obs.obs_id,'_');
    obs_id_num = str2double(obs_id_num{end});

    if mod(obs_id_num,2)==1
        obs_id = 0; % Started from Table A
    elseif mod(obs_id_num,2)==0
        obs_id = 1; % Started from Table B
    end
    
    Y((ind-1)*Nwinds+1:ind*Nwinds,2) = obs_id; % obs_id
    
end


end


% Sliding Window Mechanism to traverse the observation data one by one
function [X, Y] = sliding_window(obs)

global wind_size stride divisions numSignals;

t0 = 0;
Nwinds = ceil((1 - wind_size)/stride);


% Create Label Vector
Y = get_labels(obs, Nwinds);
X = zeros(Nwinds, divisions*numSignals);

for ind = 1:Nwinds
    tf = t0 + wind_size;
    % feature types are implemented in modular fashion
    X(ind,:) = get_features(obs,t0,tf); 
    t0 = t0 + stride;
end

end


function feats = get_features(obs, t0, tf)

global opt divisions numHapticSignals;

switch opt
    case 1
        % Raw Signals in Body Frame (except position and orientation)
        
        % Get the signal for specified window [t0,tf]
        Irft = obs.rft1.tnorm <= tf & obs.rft1.tnorm >= t0;
        Ipos = obs.pose123.tnorm <= tf & obs.pose123.tnorm >= t0;
        Iimu = obs.imu.tnorm <= tf & obs.imu.tnorm >= t0;

        % Force Torque
        f1 = obs.rft1.force(Irft,:);
        f2 = obs.rft2.force(Irft,:);
        % Total torque
        tor1 = obs.rft1.ttorque(Irft,:);
        tor2 = obs.rft2.ttorque(Irft,:);
        % Pose
        pos = obs.pose123.position(Ipos,:);
        orient = obs.imu.orientation(Ipos,:);
        % Linear Acceleration
        accel = obs.imu.pureAccel(Iimu,:);
        % Twist
        tw = obs.pose123.twistB(Ipos,:);

        % Get the means per window for listed signals
        f1_v = extract_means(f1);
        f2_v = extract_means(f2);
        tor1_v = extract_means(tor1);
        tor2_v = extract_means(tor2);

        pos_v = extract_means(pos);
        orient_v = extract_means(orient);

        accel_v = extract_means(accel);
        tw_v = extract_means(tw);
        
        % Compile them together and output
        feats = [f1_v tor1_v f2_v tor2_v pos_v orient_v tw_v accel_v ];
        
    case 2
        % Raw Signals in Spatial Frame (except position and orientation)
        
        % Get the signal for specified window [t0,tf
        Irft = obs.rft1.tnorm <= tf & obs.rft1.tnorm >= t0;
        Ipos = obs.pose123.tnorm <= tf & obs.pose123.tnorm >= t0;
        Iimu = obs.imu.tnorm <= tf & obs.imu.tnorm >= t0;

        % Force Torque
        f1 = obs.rft1.forceS(Irft,:);
        f2 = obs.rft2.forceS(Irft,:);
        % Total torque
        tor1 = obs.rft1.ttorqueS(Irft,:); 
        tor2 = obs.rft2.ttorqueS(Irft,:);

        % Pose
        pos = obs.pose123.position(Ipos,:);
        orient = obs.imu.orientation(Ipos,:);

        % Linear Acceleration
        accel = obs.imu.pureAccelS(Iimu,:);

        % Twist
        tw = obs.pose123.twistS(Ipos,:);

        % Get the means per window for listed signals
        f1_v = extract_means(f1);
        f2_v = extract_means(f2);
        tor1_v = extract_means(tor1);
        tor2_v = extract_means(tor2);

        pos_v = extract_means(pos);
        orient_v = extract_means(orient);

        accel_v = extract_means(accel);
        tw_v = extract_means(tw);
        
        % Compile them together and output
        feats = [f1_v tor1_v f2_v tor2_v pos_v orient_v tw_v accel_v ];
        
    case 3
        % Haptic features and Signals in Body frame (as in case 1 )
        
        % Get the signal for specified window [t0,tf]
        Irft = obs.rft1.tnorm <= tf & obs.rft1.tnorm >= t0;
        Ipos = obs.pose123.tnorm <= tf & obs.pose123.tnorm >= t0;
        Iimu = obs.imu.tnorm <= tf & obs.imu.tnorm >= t0;

        % Force Torque
        f1 = obs.rft1.force(Irft,:);
        f2 = obs.rft2.force(Irft,:);
        % Total torque
        tor1 = obs.rft1.ttorque(Irft,:);
        tor2 = obs.rft2.ttorque(Irft,:);
        % Pose
        pos = obs.pose123.position(Ipos,:);
        orient = obs.imu.orientation(Ipos,:);
        % Linear Acceleration
        accel = obs.imu.pureAccel(Iimu,:);
        % Twist
        tw = obs.pose123.twistB(Ipos,:);

        % Get the means per window for listed signals
        f1_v = extract_means(f1);
        f2_v = extract_means(f2);
        tor1_v = extract_means(tor1);
        tor2_v = extract_means(tor2);

        pos_v = extract_means(pos);
        orient_v = extract_means(orient);

        accel_v = extract_means(accel);
        tw_v = extract_means(tw);
        
        % Compile them together and output
        feats = [f1_v tor1_v f2_v tor2_v pos_v orient_v tw_v accel_v ];
        
        hfeats = zeros(1, divisions*numHapticSignals);
        flds = fields(obs.haptics);
        i = 1;
        for ind=1:numel(flds)
            if ~contains(flds{ind}, 'time_steps')
                hpf = obs.haptics.(flds{ind})(Irft);
                hfeats((i-1)*divisions+1 : i*divisions) = extract_means(hpf);
                i = i + 1;
            end
        end
        feats = [feats hfeats];
        
    case 4
        % Haptic features and Signals in Spatial frame (as in case 1 )
        
        % Get the signal for specified window [t0,tf]
        Irft = obs.rft1.tnorm <= tf & obs.rft1.tnorm >= t0;
        Ipos = obs.pose123.tnorm <= tf & obs.pose123.tnorm >= t0;
        Iimu = obs.imu.tnorm <= tf & obs.imu.tnorm >= t0;

        % Force Torque
        f1 = obs.rft1.forceS(Irft,:);
        f2 = obs.rft2.forceS(Irft,:);
        % Total torque
        tor1 = obs.rft1.ttorqueS(Irft,:); 
        tor2 = obs.rft2.ttorqueS(Irft,:);
        % Pose
        pos = obs.pose123.position(Ipos,:);
        orient = obs.imu.orientation(Ipos,:);
        % Linear Acceleration
        accel = obs.imu.pureAccelS(Iimu,:);
        % Twist
        tw = obs.pose123.twistS(Ipos,:);

        % Get the means per window for listed signals
        f1_v = extract_means(f1);
        f2_v = extract_means(f2);
        tor1_v = extract_means(tor1);
        tor2_v = extract_means(tor2);

        pos_v = extract_means(pos);
        orient_v = extract_means(orient);

        accel_v = extract_means(accel);
        tw_v = extract_means(tw);
        
        % Compile them together and output
        feats = [f1_v tor1_v f2_v tor2_v pos_v orient_v tw_v accel_v ];
        
        hfeats = zeros(1, divisions*numHapticSignals);
        flds = fields(obs.haptics);
        i = 1;
        for ind=1:numel(flds)
            if ~contains(flds{ind}, 'time_steps')
                hpf = obs.haptics.(flds{ind})(Irft);
                hfeats((i-1)*divisions+1 : i*divisions) = extract_means(hpf);
                i = i + 1;
            end
        end
        feats = [feats hfeats];
        
    case 5
        % Pure Haptic Signals
        
        % Get the signal for specified window [t0,tf]
        Irft = obs.rft1.tnorm <= tf & obs.rft1.tnorm >= t0;
        
        feats = zeros(1, divisions*numHapticSignals);
        flds = fields(obs.haptics);
        i = 1;
        for ind=1:numel(flds)
            if ~contains(flds{ind}, 'time_steps')
                hpf = obs.haptics.(flds{ind})(Irft);
                feats((i-1)*divisions+1 : i*divisions) = extract_means(hpf);
                i = i + 1;
            end
        end
        
end
end


% Get mean values of the signal over divions within the window
function mvec = extract_means(x)
global divisions

    [len,dim] = size(x);
    
    sub_div = len/divisions;

    mvec = zeros(divisions,dim);
    
    for i=1:divisions
        t1 = floor((i-1)*sub_div)+1;
        t2 = floor(i*sub_div);
        mvec(i,:) = mean(x(t1:t2,:));
    end

    mvec = reshape(mvec',1, numel(mvec));

end


function Y = get_labels(obs, Nwinds)

    Y = zeros(Nwinds, 3);
    Y(:,1) = 1:Nwinds; % index for each window within entire observation

    if strcmp(obs.traj_type,'AB1')
        Y(:,2) = 0;
    else
        Y(:,2) = 1;
    end

    if strcmp(obs.motion_type,'serial')
        Y(:,3) = 0;
    elseif strcmp(obs.motion_type,'parallel')
        Y(:,3) = 1;
    elseif strcmp(obs.motion_type,'serial->parallel')
        Y(:,3) = 3;
    end

end





