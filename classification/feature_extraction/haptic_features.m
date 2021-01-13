function [feat] = haptic_features(feat,obs)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

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


feat.haptics.(sprintf('Me1f%d%d',ax)) = vecdot(obs.rft1.force(:,ax),obs.fsum.force(:,ax))...
 ./(vecnorm(obs.rft1.force(:,ax),2,2).*vecnorm(obs.fsum.force(:,ax),2,2));

feat.haptics.(sprintf('Me2f%d%d',ax)) = vecdot(obs.rft2.force(:,ax),obs.fsum.force(:,ax))...
 ./(vecnorm(obs.rft2.force(:,ax),2,2).*vecnorm(obs.fsum.force(:,ax),2,2));


feat.haptics.(sprintf('Mtf%d%d',ax)) = vecnorm(obs.fsum.force(:,ax),2,2)...
    ./(vecnorm(obs.rft1.force(:,ax),2,2)+vecnorm(obs.rft2.force(:,ax),2,2));

feat.haptics.(sprintf('Mnf%d%d',ax)) = vecnorm(obs.fsum.force(:,ax),2,2).^2 ...
    ./(abs(vecdot(obs.rft1.force(:,ax),obs.fsum.force(:,ax)))...
     + abs(vecdot(obs.rft2.force(:,ax),obs.fsum.force(:,ax))));

feat.haptics.(sprintf('Msf%d%d',ax)) = 1 - abs( (abs(vecdot(obs.rft1.force(:,ax),obs.fsum.force(:,ax)))...
     - abs(vecdot(obs.rft2.force(:,ax),obs.fsum.force(:,ax)))) ...
     ./vecnorm(obs.fsum.force(:,ax),2,2).^2);
end


function feat = torque_components(feat, obs, ax)


feat.haptics.(sprintf('Me1t%d%d',ax)) = vecdot(obs.rft1.ttorque(:,ax),obs.fsum.ttsum(:,ax))...
     ./(vecnorm(obs.rft1.ttorque(:,ax),2,2).*vecnorm(obs.fsum.ttsum(:,ax),2,2));

feat.haptics.(sprintf('Me2t%d%d',ax)) = vecdot(obs.rft2.ttorque(:,ax),obs.fsum.ttsum(:,ax))...
     ./(vecnorm(obs.rft2.ttorque(:,ax),2,2).*vecnorm(obs.fsum.ttsum(:,ax),2,2));


feat.haptics.(sprintf('Mtt%d%d',ax)) = vecnorm(obs.fsum.ttsum(:,ax),2,2)...
     ./(vecnorm(obs.rft1.ttorque(:,ax),2,2)+vecnorm(obs.rft2.ttorque(:,ax),2,2));

feat.haptics.(sprintf('Mnt%d%d',ax)) = vecnorm(obs.fsum.ttsum(:,ax),2,2).^2 ...
     ./(abs(vecdot(obs.rft1.ttorque(:,ax),obs.fsum.ttsum(:,ax)))...
     + abs(vecdot(obs.rft2.ttorque(:,ax),obs.fsum.ttsum(:,ax))));

feat.haptics.(sprintf('Mst%d%d',ax)) = 1 - abs( (abs(vecdot(obs.rft1.ttorque(:,ax),obs.fsum.ttsum(:,ax)))...
     - abs(vecdot(obs.rft2.ttorque(:,ax),obs.fsum.ttsum(:,ax)))) ...
     ./vecnorm(obs.fsum.ttsum(:,ax),2,2).^2);
 
end






