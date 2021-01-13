function [z,y] = integrate_features(feat,t0,tf)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here


    fns = fieldnames(feat.haptics);
    z = zeros(1,numel(fns)-1);
    ind = 1;
    for i=1:numel(fns)
       if ~strcmp(fns{i}, 'time_steps') 
            I = feat.haptics.time_steps>=t0 & feat.haptics.time_steps<=tf;

            x = feat.haptics.time_steps(I);
            y = feat.haptics.(fns{i})(I);

            z(ind) = trapz(x,y);
            ind = ind+1;
       end
    end
    
    if strcmp(feat.traj_type,'AB1')
        y = 0;
    elseif strcmp(feat.traj_type,'AB2')
        y = 1;
    end
     
    
end

