function [primitives, primtable, N] = get_primitives(observations_processed, DetectedEvents)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% global weight
% weight = 24; % Weight of the tray

st = struct2table(DetectedEvents);
st = convertvars(st, {'obs_id'}, 'string');
M = containers.Map(st.obs_id, st.events);

% Count total number of events
N = 0;
for i = 1:numel(DetectedEvents)
   N = N + size(DetectedEvents(i).events,1);
end

varnames =  {'obs_id','obs_ind', 'seqnum',...
    'lfcons', 'schange', 'fstr_max', 'fstr_min', ...
    'fstr_mean', 'fstr_t_extr', 'fsum_max', ...
    'fsum_min', 'fsum_mean', 'exp_dir',...
    'outcome', 'vmax', 'vy_gain', 'vy_mean', 'inst_dec'};

% varnames =  {'obs_id','obs_ind', 'seqnum',...
%     'lfcons', 'schange', 'fstr_max_x', 'fstr_max_y', 'fstr_min_x', 'fstr_min_y', ...
%     'fstr_mean_x', 'fstr_mean_y', 'fstr_t_extr', 'fsum_max_x', 'fsum_max_y', ...
%     'fsum_min_x', 'fsum_min_y', 'fsum_mean_x', 'fsum_mean_y', 'exp_dir',...
%     'outcome', 'vmax_x', 'vmax_y', 'vy_gain', 'vy_mean', 'inst_dec'};
% primtable = array2table(zeros(N, numel(varnames)), 'VariableNames', varnames);



% Initialize primitives struct
primitives(N) = struct('prim', [], 'prim_id', [], 'obs_id', [], ...
    'obs_ind', [], 'inst_dec', [], 'is_bf', [], 'dec_prim', [], 'seqnum', [],...
    'outcome',  [], 'exp_dir', [], ...
    'lfcons', [], 'schange', [], 'fstr_max', [], 'fstr_min', [], ...
    'fstr_mean', [], 'fstr_t_extr', [],  'fstr_std', [], ...
    'fsum_min', [], 'fsum_mean', [], 'fsum_max', [], 'fsum_std', [],  ...
    'angs_max', [], 'angs_min', [], 'angs_std', [], 'angs_mean', [], ...
    'ang_diff_max', [], 'ang_diff_min', [], 'ang_diff_mean', [], 'ang_diff_std', [], ...
    'f1_max',[], 'f1_min',[], 'f1_mean',[], 'f2_max',[],...
    'f2_min',[], 'f2_mean',[], 'f1_std',[], 'f2_std', [],  ...
    'tau1_max',[], 'tau1_min',[], 'tau1_mean',[], 'tau2_max',[], ...
    'tau2_min',[], 'tau2_mean',[], 'tau1_std',[], 'tau2_std',[], ...
    'tau_sum_max', [], 'tau_sum_mean', [], 'tau_sum_min', [], 'tau_sum_std',[], ...
    'tau_str_max', [], 'tau_str_mean', [], 'tau_str_min', [], 'tau_str_std', [], ...
    'orient_max', [], 'orient_min', [], 'orient_mean',[], 'orient_std',[], ...
    'pos_max', [], 'pos_min', [], 'pos_mean',[], 'pos_std',[], ...
    'angvel_max', [], 'angvel_min', [], 'angvel_mean', [], 'angvel_std',[], ...
    'vmax', [],'vmin', [], 'v_gain', [], 'v_mean', [], 'v_std', []);


[features] = haptic_features(observations_processed);


ev_ind = 0;
for obs_ind = progress(1:numel(DetectedEvents), 'Title', 'GettingPrimitives')
    obs = observations_processed(obs_ind);
    feat = features(obs_ind);
    
    ev_mat = M(obs.obs_id);
    for seqnum = 1:size(ev_mat,1)
        
        ev_ind = ev_ind+1;
        
        ev = ev_mat(seqnum,:);
        primitives(ev_ind).prim = get_prim(obs,feat,ev);
        
        primitives(ev_ind).seqnum = seqnum;
        primitives(ev_ind).obs_id = obs.obs_id;
        primitives(ev_ind).obs_ind = obs_ind;
        primitives(ev_ind).prim_id = ev_ind;
        primitives(ev_ind).duration = ev(2)-ev(1);
        
        primitives(ev_ind) = get_prim_info(primitives(ev_ind), obs);
        
        
    end
end

primtable = struct2table(primitives);

primtable = convertvars(primtable, 'obs_id', 'string');

primtable.is_bf = is_bf(primtable);
primtable.dec_prim = dec_prim(primtable);
end



function [prim] = get_prim(obs,feat,win_loc)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% global weight
% Time frame
Ipos = obs.pose123.time_steps >= win_loc(1) & obs.pose123.time_steps <= win_loc(2);
Irft = obs.rft1.time_steps >= win_loc(1) & obs.rft1.time_steps <= win_loc(2);

t_ev = obs.pose123.time_steps(Ipos);

prim.time_steps = t_ev;

% Get signals
prim.angles = feat.angles.angles(Ipos, 1:3);
ang_diff = feat.angles.angles(Ipos, 2) - feat.angles.angles(Ipos, 3);
prim.ang_diff = ang_diff;


fstr = obs.fstretch.force(Irft, 1);
prim.fstr = fstr(1:10:end,:);
% rotm = axang2rotm(obs.imu.orientation(Ipos,:));
% pitch_cos = rotm(3,3,:);
% prim.fstr = prim.fstr.*pitch_cos(1,:)'; % compensation for gravity due to pitch angle

fsum = obs.fsum.force(Irft, 1:2);
prim.fsum = fsum(1:10:end,:);

f1 = obs.rft1.force(Irft,1:2);
f2 = obs.rft2.force(Irft,1:2);
prim.f1 = f1(1:10:end,:);
prim.f2 = f2(1:10:end,:);

tau1 = obs.rft1.torque(Irft,3);
tau2 = obs.rft2.force(Irft,3);
prim.tau1 = tau1(1:10:end);
prim.tau2 = tau2(1:10:end);

prim.tau_sum = prim.tau1 + prim.tau2;
prim.tau_str = prim.tau1 - prim.tau2;

prim.vel = obs.pose123.linvel(Ipos, 1:2);

prim.angvel = obs.imu.angvelS(Ipos, 3);

prim.pos = obs.pose123.position(Ipos,1:2);

prim.orient = obs.imu.orientation(Ipos,4);
prim.orient = prim.orient - prim.orient(1);

prim.accel = obs.imu.accelS(Ipos, :);

prim.obs_id = obs.obs_id;

prim.duration = win_loc(2) - win_loc(1);

end


function res = get_prim_info(res, obs)
    prim = res.prim;
    ang = prim.angles(:,2) - prim.angles(:, 3);

    % Variables of interest
    lfcons = sum(ang>0)/numel(ang) + 1.; % closer to 1 - F1 is in leader

    pos = ang>0;
    changes = xor(pos(1:end-1),pos(2:end));
    schange = sum(changes);
    
    [a,b] = max(abs(prim.fstr(:,1)));
    fstr_t_extr = prim.time_steps(b) - prim.time_steps(1);
    
    
    [a,b] = max(abs(prim.vel(:,2)));
    vel_extr = prim.vel(b,2);
    exp_dir = 0; % Neutral
    if abs(vel_extr) >= 0.05
        % vy > 0 - left
        % exp_dir= -1 - left
        % exp_dir= 1 - right 
        exp_dir = -sign(vel_extr); 
    end

    outcome = 1;
    if strcmp(obs.traj_type, 'AB1') % left
        outcome = -1;
    end

    
    res.lfcons = lfcons;
    res.schange = schange;
    
    res.angs_max = max(prim.angles);
    res.angs_min = min(prim.angles);
    res.angs_mean = mean(prim.angles);
    res.angs_std = std(prim.angles);
    
    res.ang_diff_max = max(prim.ang_diff);
    res.ang_diff_min = min(prim.ang_diff);
    res.ang_diff_mean = mean(prim.ang_diff);
    res.ang_diff_std = std(prim.ang_diff);
    
    res.fstr_max = max(prim.fstr);
    res.fstr_min = min(prim.fstr);
    res.fstr_mean = mean(prim.fstr);
    res.fstr_std = std(prim.fstr);
    
    res.fstr_t_extr = fstr_t_extr;
    res.fsum_max = max(prim.fsum);
    res.fsum_min = min(prim.fsum);
    res.fsum_mean = mean(prim.fsum);
    res.fsum_std = std(prim.fsum);
    
    res.f1_max = max(prim.f1);
    res.f1_min = min(prim.f1);
    res.f1_mean = mean(prim.f1);
    res.f1_std = std(prim.f1);
    res.f2_max = max(prim.f2);
    res.f2_min = min(prim.f2);
    res.f2_mean = mean(prim.f2);
    res.f2_std = std(prim.f2);
    
    res.tau1_max = max(prim.tau1);
    res.tau1_min = min(prim.tau1);
    res.tau1_mean = mean(prim.tau1);
    res.tau1_std = std(prim.tau1);
    res.tau2_std = std(prim.tau2);
    res.tau2_max = max(prim.tau2);
    res.tau2_min = min(prim.tau2);
    res.tau2_mean = mean(prim.tau2);
    
    res.tau_sum_max = max(prim.tau_sum);
    res.tau_sum_min = min(prim.tau_sum);
    res.tau_sum_mean = mean(prim.tau_sum);
    res.tau_sum_std = std(prim.tau_sum);
    res.tau_str_max = max(prim.tau_str);
    res.tau_str_min = min(prim.tau_str);
    res.tau_str_mean = mean(prim.tau_str);
    res.tau_str_std = std(prim.tau_str);
    
    res.exp_dir = exp_dir;
    res.outcome = outcome;
    
    res.vmax = max(prim.vel);
    res.vmin = min(prim.vel);
    res.v_gain = prim.vel(end,:) - prim.vel(1,:);
    res.v_mean = mean(prim.vel);
    res.v_std = std(prim.vel);
    
    res.orient_max = max(prim.orient);
    res.orient_min = min(prim.orient);
    res.orient_mean = mean(prim.orient);
    res.orient_std = std(prim.orient);
    
    res.angvel_max = max(prim.angvel);
    res.angvel_min = min(prim.angvel);
    res.angvel_mean = mean(prim.angvel);
    res.angvel_std = std(prim.angvel);
    
    res.pos_max = max(prim.pos);
    res.pos_min = min(prim.pos);
    res.pos_mean = mean(prim.pos);
    res.pos_std = std(prim.pos);
    
    res.inst_dec = double(obs.inst_dec);

end


function arr = is_bf(primtable)
    
    arr = zeros([height(primtable),1]);
    for ind = 1:max(primtable.obs_ind)
       arr(primtable.obs_ind==ind) = sum(diff(primtable.exp_dir(primtable.obs_ind==ind))~=0);
    end

end

function arr = dec_prim(primtable)
    
    arr = zeros([height(primtable),1]);
    for ind = 1:max(primtable.obs_ind)
       exp_dir = primtable.exp_dir(primtable.obs_ind==ind);
       arr(primtable.obs_ind==ind) = get_dec_prim_idx(exp_dir);
    end

end


function dec_prim = get_dec_prim_idx(exp_dir)
    dec_prim = zeros(size(exp_dir));
    
    if numel(exp_dir)==1
        dec_prim = 1; 
        return
    end
    
    s_term = exp_dir(end);
    for i=numel(exp_dir)-1:-1:1
        if exp_dir(i)~=s_term
            dec_prim(i+1) = 1;
            return
        end
    end
    dec_prim(1) = 1;
end







