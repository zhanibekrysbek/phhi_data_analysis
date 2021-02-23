function [X,Y] = extractSWFeatures(observations_processed, tb, option)
%extractSWFeatures Summary of this function goes here
%   Output: 
%       X - data matrix: 
%           rows - observations, columns - features
%       Y - label: 
%           Y = [obs_id starting_table window_num traj_type motion_type 
%               initialOrientation outcomeSubject]
%           
%       data_option:
%               1 - Raw Signals in Body Frame (except position and orientation)
%                   28 separate signals
%               2 - Raw Signals in Spatial Frame (except position and orientation)
%                   28 separate signals
%               3 - Haptic features and Signals in Body frame (as in case 1 )
%                   58 separate signals
%               4 - Haptic features and Signals in Spatial frame (as in case 1 )
%                   58 separate signals
%               5 - Pure Haptic Signals
%                   30 separate signals
%               6 - 
%                   Sliding Windwos until tdec_sec
%
%



global wind_size stride divisions labelSize opt numHapticSignals numSignals;



labelSize = 7; % maximum number of labels
opt = option; % Option for feature data
opt2numsig = [28, 28, 38, 38, 10, 29]; % number of signals

numSignals = opt2numsig(opt); % number of signals
numHapticSignals = 10; % number of Haptic signals


if opt <=5
    [X, Y] = SWNormalized_time(observations_processed);
elseif opt <= numel(opt2numsig) && opt >=6
    [X, Y] = SWDec_time(observations_processed, tb);
else
    error("Wrong input for option: %d", opt);
end

end





function [X, Y] = SWDec_time(observations_processed, tb)
% A function to extract sliding window features from entire observations
% until TDEC_SEC (a moment when left/right decision is made).

global Nwinds wind_size stride divisions opt numHapticSignals labelSize numSignals;



hfeatures = haptic_features(observations_processed);

wind_size = 0.1;
stride = 0.06;
divisions = 1;

tdec_sec = round(tb.tdec_sec,2);
Nwinds_arr = ceil((tdec_sec - wind_size)/stride);


X = zeros(sum(Nwinds_arr), divisions*numSignals);
Y = zeros(sum(Nwinds_arr), labelSize);

left = 1;

% loop through all observations
for ind=progress(1:numel(observations_processed), 'Title', 'ExtractingSWFeatures')
    
    obs = observations_processed(ind);
    
    obs.angles = hfeatures(ind).angles;
    obs.integs = hfeatures(ind).integs;
    
    right = left + Nwinds_arr(ind)-1;
    Nwinds = Nwinds_arr(ind);
    % get the values by sliding window
%     Irange = (ind-1)*Nwinds+1 : ind*Nwinds;
    Irange = left : right;
    [X(Irange,:), Y(Irange,3:end)] = sliding_window(obs);
      
    Y(Irange,1) = ind; % obs_id
    
    % Identify Initial location (Table A or Table B)
    obs_id_num = split(obs.obs_id,'_');
    obs_id_num = str2double(obs_id_num{end});

    if mod(obs_id_num,2)==1
        table_loc = 1; % Started from Table A
    elseif mod(obs_id_num,2)==0
        table_loc = 2; % Started from Table B
    end
    
    Y(Irange,2) = table_loc; % obs_id
    
%     fprintf("range: %d %d\n", left, right);
    
    left = right+1;
end




end



function [X, Y] = SWNormalized_time(observations_processed)

% A function to extract sliding window features from entire observations
% with NORMALIZED TIME.

global Nwinds wind_size stride divisions opt numHapticSignals numSignals;

wind_size = 0.05;
stride = 0.02;
divisions = 5;

if opt >= 3 
   hfeatures = haptic_features(observations_processed);
end

Nwinds = ceil((1 - wind_size)/stride);

X = zeros(Nwinds*numel(observations_processed), divisions*numSignals);
Y = zeros(Nwinds*numel(observations_processed), 7);

% loop through all observations
for ind=progress(1:numel(observations_processed), 'Title', 'ExtractingSWFeatures')
    
    obs = observations_processed(ind);
    if opt >= 3
       obs.haptics = hfeatures(ind).haptics;
    end
    
    % get the values by sliding window
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

global Nwinds wind_size stride divisions numSignals;


t0 = 0;
% Nwinds = ceil((1 - wind_size)/stride);


% Create Label and Feature Vector
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
        
    case 6
        % Raw Signals in Body Frame (except position and orientation)
        
        % Get the signal for specified window [t0,tf]
        Irft = obs.rft1.time_steps <= tf & obs.rft1.time_steps >= t0;
        Ipos = obs.pose123.time_steps <= tf & obs.pose123.time_steps >= t0;
        Iimu = obs.imu.time_steps <= tf & obs.imu.time_steps >= t0;

        % Force Torque
        f1 = obs.rft1.force(Irft,1:2);
        f2 = obs.rft2.force(Irft,1:2);
        fstretch = obs.fstretch.force(Irft, 1:2);
        fsum = obs.fsum.force(Irft, :);
        
        % Total torque
        tor1 = obs.rft1.ttorque(Irft,3);
        tor2 = obs.rft2.ttorque(Irft,3);
        
        % Pose
        pos = obs.pose123.position(Ipos,1:2);
        
        orient = rad2deg(obs.imu.orientation(Ipos,end));
        orient = orient - orient(1);
        
        % Hfeatures
        angles = obs.angles.angles(Ipos, :);
        integs = obs.integs.integs(Ipos,:);
        
        % Twist
        tw = obs.pose123.twistB(Ipos,[1,2,6]);

        % Get the means per window for listed signals
        f1_v = extract_means(f1);
        f2_v = extract_means(f2);
        fstretch_v = extract_means(fstretch);
        fsum_v = extract_means(fsum);
        tor1_v = extract_means(tor1);
        tor2_v = extract_means(tor2);

        pos_v = extract_means(pos);
        orient_v = extract_means(orient);

        tw_v = extract_means(tw);
        
        angles_v = extract_means(angles);
        integs_v = extract_means(integs);
        
        % Compile them together and output
        feats = [f1_v tor1_v f2_v tor2_v fstretch_v fsum_v pos_v orient_v ...
            tw_v angles_v integs_v];
        
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

    Y = zeros(Nwinds, 5);
    Y(:,1) = 1:Nwinds; % index for each window within entire observation

    if strcmp(obs.traj_type,'AB1')
        Y(:,2) = 1;
    else
        Y(:,2) = 2;
    end

    if strcmp(obs.motion_type,'serial')
        Y(:,3) = 1;
    elseif strcmp(obs.motion_type,'parallel')
        Y(:,3) = 2;
    elseif strcmp(obs.motion_type,'serial->parallel')
        Y(:,3) = 3;
    end
    
    if strcmp(obs.initialOrient,'xr')
        Y(:,4) = 1;
    else
        Y(:,4) = 2;
    end
    
    Y(:,5) = obs.outcomeSubject;

end





