function [features] = haptic_features(observations_processed)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


features(numel(observations_processed)) = struct('haptics',[], 'angles',[],...
    'integs',[], 'motion_type',[],'obs_id',[], 'traj_type', []);
        
% profile on;
for i=progress(1:numel(features), 'Title','HapticFeatures')
    
    obs = observations_processed(i);
    features(i).obs_id = obs.obs_id;
    features(i).traj_type = obs.traj_type;
    features(i).motion_type = obs.motion_type;
    
    features(i) = get_indices(features(i), obs);
    features(i) = get_angles(features(i), obs);
    features(i) = get_integrals(features(i), obs);
     
end

end


function feat = get_integrals(feat,obs)

fstr = obs.fstretch.force(1:10:end,1);

integs = zeros(numel(fstr),2);
integs(:,1) = cumtrapz(obs.pose123.time_steps,fstr);
integs(:,2) = cumtrapz(obs.pose123.position(:,1),fstr);

feat.integs.integs = integs;
feat.integs.time_steps = obs.pose123.time_steps;
end


function feat = get_angles(feat,obs)

% vectors of interest
v = obs.pose123.linvel(:,[1,2]);
f1 = obs.rft1.forceS(1:10:end,1:2);
f2 = obs.rft2.forceS(1:10:end,1:2);
fsum = obs.fsum.forceS(1:10:end,1:2);
fstr = obs.fstretch.forceS(1:10:end,1:2);

% get the angles between the vectors of interest in degrees
theta = zeros(numel(obs.pose123.time_steps), 10);

theta(:,1) = acosd(vecdot(f1,f2)./(vecnorm(f1,2,2).*vecnorm(f2,2,2)));
theta(:,2) = acosd(vecdot(f1,v)./(vecnorm(f1,2,2).*vecnorm(v,2,2)));
theta(:,3) = acosd(vecdot(f2,v)./(vecnorm(f2,2,2).*vecnorm(v,2,2)));
theta(:,4) = acosd(vecdot(fsum,v)./(vecnorm(fsum,2,2).*vecnorm(v,2,2)));
theta(:,5) = acosd(vecdot(fstr,v)./(vecnorm(fstr,2,2).*vecnorm(v,2,2)));
theta(:,6) = acosd(vecdot(f1,fsum)./(vecnorm(f1,2,2).*vecnorm(fsum,2,2)));
theta(:,7) = acosd(vecdot(f2,fsum)./(vecnorm(f2,2,2).*vecnorm(fsum,2,2)));
theta(:,8) = acosd(vecdot(fstr,fsum)./(vecnorm(fstr,2,2).*vecnorm(fsum,2,2)));
theta(:,9) = acosd(vecdot(f1,fstr)./(vecnorm(f1,2,2).*vecnorm(fstr,2,2)));
theta(:,10) = acosd(vecdot(f2,fstr)./(vecnorm(f2,2,2).*vecnorm(fstr,2,2)));

feat.angles.angles = theta;
feat.angles.time_steps = obs.pose123.time_steps;
end



function feat = get_indices(feat,obs)

% axs = [1, 2;
%        1, 3;
%        2, 3];
axs = [1, 2];

for i=1:height(axs)
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






