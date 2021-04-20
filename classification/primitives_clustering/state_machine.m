


%% Obtain state transition matrix


% idx_temp(primtable.exec_prim==1) = numel(ks)+1;
% primtable.cluster = idx_temp';

% primtable.cluster(primtable.exec_prim==1) = k+1;

tb_temp = primtable(primtable.obs_ind ~= 86 & primtable.obs_ind~=102, : );

[Stable, S, Snorm] = transition_mat(tb_temp);


%% State Machine for the first trials


% idx_temp(primtable.exec_prim==1) = numel(ks)+1;
% primtable.cluster = idx_temp';

% primtable.cluster(primtable.exec_prim==1) = k+1;

thresh = 5;

% Get trajectory indices per dyad
trial_inds = zeros([size(primtable,1),1]);
for ind = 1:size(primtable,1)
    temp = split(primtable.obs_id(ind),'_');
    trial_inds(ind) = str2double(temp(end));
end
primtable.trial_inds = trial_inds;

tb_temp = primtable(primtable.trial_inds <=thresh,:);


[Stable1, S1, Snorm1] = transition_mat(tb_temp);



%% State Machine for a later trials


% idx_temp(primtable.exec_prim==1) = numel(ks)+1;
% primtable.cluster = idx_temp';

% primtable.cluster(primtable.exec_prim==1) = k+1;

thresh = 5;

% Get trajectory indices per dyad
trial_inds = zeros([size(primtable,1),1]);
for ind = 1:size(primtable,1)
    temp = split(primtable.obs_id(ind),'_');
    trial_inds(ind) = str2double(temp(end));
end
primtable.trial_inds = trial_inds;

tb_temp = primtable(primtable.trial_inds > thresh,:);

[Stable2, S2, Snorm2] = transition_mat(tb_temp);


%% State Machine with/without Vignesh


% idx_temp(primtable.exec_prim==1) = numel(ks)+1;
% primtable.cluster = idx_temp';

% primtable.cluster(primtable.exec_prim==1) = k+1;


tb_temp = primtable(contains(primtable.obs_id,'Vignesh'),:);
[Stable3, S3, Snorm3] = transition_mat(tb_temp);

tb_temp = primtable(~contains(primtable.obs_id,'Vignesh'),:);
[Stable4, S4, Snorm4] = transition_mat(tb_temp);

