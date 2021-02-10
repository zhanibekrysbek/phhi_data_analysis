function [features] = haptic_features(observations_processed)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


features(numel(observations_processed)) = struct('haptics',[], 'motion_type',[],...
            'obs_id',[], 'traj_type', []);
        
% profile on;
for i=progress(1:numel(features), 'Title','HapticFeatures')
    
    obs = observations_processed(i);
    features(i).obs_id = obs.obs_id;
    features(i).traj_type = obs.traj_type;
    features(i).motion_type = obs.motion_type;
    
    features(i) = get_indices(features(i), obs);
     
end


end

function feat = get_indices(feat,obs)

axs = [1, 2;
       1, 3;
       2, 3];

for i=1:length(axs)
    feat = force_components(feat, obs, axs(i,:));
    feat = torque_components(feat, obs, axs(i,:));
end

feat.haptics.time_steps = obs.rft1.time_steps;
end




function feat = force_components(feat, obs, ax)

% Individual Efficiency
feat.haptics.(sprintf('Me1f%d%d',ax)) = vecdot(obs.rft1.force(:,ax),obs.fsum.force(:,ax))...
 ./(vecnorm(obs.rft1.force(:,ax),2,2).*vecnorm(obs.fsum.force(:,ax),2,2));

% Individual Efficiency
feat.haptics.(sprintf('Me2f%d%d',ax)) = vecdot(obs.rft2.force(:,ax),obs.fsum.force(:,ax))...
 ./(vecnorm(obs.rft2.force(:,ax),2,2).*vecnorm(obs.fsum.force(:,ax),2,2));

% Team Efficiency
feat.haptics.(sprintf('Mtf%d%d',ax)) = vecnorm(obs.fsum.force(:,ax),2,2)...
    ./(vecnorm(obs.rft1.force(:,ax),2,2)+vecnorm(obs.rft2.force(:,ax),2,2));

% Negotiation Efficiency
feat.haptics.(sprintf('Mnf%d%d',ax)) = vecnorm(obs.fsum.force(:,ax),2,2).^2 ...
    ./(abs(vecdot(obs.rft1.force(:,ax),obs.fsum.force(:,ax)))...
     + abs(vecdot(obs.rft2.force(:,ax),obs.fsum.force(:,ax))));
% Similarity of Forces
feat.haptics.(sprintf('Msf%d%d',ax)) = 1 - abs( (abs(vecdot(obs.rft1.force(:,ax),obs.fsum.force(:,ax)))...
     - abs(vecdot(obs.rft2.force(:,ax),obs.fsum.force(:,ax)))) ...
     ./vecnorm(obs.fsum.force(:,ax),2,2).^2);
end


function feat = torque_components(feat, obs, ax)

% Individual Efficiency
feat.haptics.(sprintf('Me1t%d%d',ax)) = vecdot(obs.rft1.ttorque(:,ax),obs.fsum.ttsum(:,ax))...
     ./(vecnorm(obs.rft1.ttorque(:,ax),2,2).*vecnorm(obs.fsum.ttsum(:,ax),2,2));

% Individual Efficiency
feat.haptics.(sprintf('Me2t%d%d',ax)) = vecdot(obs.rft2.ttorque(:,ax),obs.fsum.ttsum(:,ax))...
     ./(vecnorm(obs.rft2.ttorque(:,ax),2,2).*vecnorm(obs.fsum.ttsum(:,ax),2,2));

% Team Efficiency
feat.haptics.(sprintf('Mtt%d%d',ax)) = vecnorm(obs.fsum.ttsum(:,ax),2,2)...
     ./(vecnorm(obs.rft1.ttorque(:,ax),2,2)+vecnorm(obs.rft2.ttorque(:,ax),2,2));

% Negotiation Efficiency
feat.haptics.(sprintf('Mnt%d%d',ax)) = vecnorm(obs.fsum.ttsum(:,ax),2,2).^2 ...
     ./(abs(vecdot(obs.rft1.ttorque(:,ax),obs.fsum.ttsum(:,ax)))...
     + abs(vecdot(obs.rft2.ttorque(:,ax),obs.fsum.ttsum(:,ax))));

% Similarity of Forces
feat.haptics.(sprintf('Mst%d%d',ax)) = 1 - abs( (abs(vecdot(obs.rft1.ttorque(:,ax),obs.fsum.ttsum(:,ax)))...
     - abs(vecdot(obs.rft2.ttorque(:,ax),obs.fsum.ttsum(:,ax)))) ...
     ./vecnorm(obs.fsum.ttsum(:,ax),2,2).^2);
 
end






