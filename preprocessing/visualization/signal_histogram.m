function [hist_mat_f,hist_mat_v, fstr_label, vel_label, tlabel] = signal_histogram(sl)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

fstr_max = 25;
fstr_min = -25;
fstr_res = linspace(fstr_min, fstr_max, 41);
vel_res = linspace(-0.7,0.7,41);
tres = linspace(-0.05, 1.01, 41);

fstr_label = get_fstr_label(fstr_res);
vel_label = get_vel_label(vel_res);
tlabel = get_tlabel(tres);

hist_mat_f = zeros(numel(fstr_res)-1, numel(tres)-1);
hist_mat_v = zeros(numel(vel_res)-1, numel(tres)-1);

% Get Fstretch Done
% Nested loop for one matrix
for ind2=1:size(hist_mat_f,2)-1 % loop over Fstr
    
    tstr_bin = tres(ind2:ind2+1);
    
    for ind1=1:size(hist_mat_f,1)-1 % loop over tres
        
        fstr_bin = fstr_res(ind1:ind1+1);
        
        % Go over primitives
        for indp = 1:size(sl,1)
            prim = sl.prim(indp); 
            
            t = prim.time_steps - prim.time_steps(1);
            t = t/t(end);
            
            % suppress extreme values
            fstr = prim.fstr;
            if max(fstr) > fstr_max
                fstr = fstr/max(fstr)*(fstr_max-1);
            end

            if min(fstr) < fstr_min
                fstr = fstr/min(fstr)*(fstr_min+1);
            end

            tinds = t>=tstr_bin(1) & t<tstr_bin(2);
            finds = fstr(tinds) >= fstr_bin(1) & fstr(tinds) < fstr_bin(2);
            
            
            if sum(finds) ~= 0
                hist_mat_f(ind1,ind2) = hist_mat_f(ind1,ind2) + 1;
            end
             
        end
    end

end




% Get Fstretch Done
% Nested loop for one matrix
for ind2=1:size(hist_mat_v,2)-1 % loop over Fstr
    
    tstr_bin = tres(ind2:ind2+1);
    
    for ind1=1:size(hist_mat_v,1)-1 % loop over tres
        
        vel_bin = vel_res(ind1:ind1+1);
        
        % Go over primitives
        for indp = 1:size(sl,1)
            prim = sl.prim(indp); 
            
            t = prim.time_steps - prim.time_steps(1);
            t = t/t(end);
            
            vel = prim.vel(:,2);

            tinds = t>=tstr_bin(1) & t<tstr_bin(2);
            vinds = vel(tinds) >= vel_bin(1) & vel(tinds) < vel_bin(2);
            
            
            if sum(vinds) ~= 0
                hist_mat_v(ind1,ind2) = hist_mat_v(ind1,ind2) + 1;
            end
             
        end
    end

end




hist_mat_f = flip(hist_mat_f);
hist_mat_v = flip(hist_mat_v);


end


% Algorithm 1. Traverses over trajectories
% for i=1:size(sl,1)
%     prim = sl.prim(i); 
%     t = prim.time_steps - prim.time_steps(1);
%     t = t/t(end);
%     
%     fstr = prim.fstr;
%     if max(fstr) > fstr_max
%         fstr = fstr/max(fstr)*(fstr_max-1);
%     end
%     
%     if min(fstr) < fstr_min
%         fstr = fstr/min(fstr)*(fstr_min+1);
%     end
%     
%     % run over sampled points
%     for j=1:numel(t)
%         % corressponding bin numbers
%         idx_f = find(fstr_res <= fstr(j),1,'last');
%         idx_t = find(tres <= t(j),1,'last');
%         idx_v = find(vel_res <= prim.vel(j,2),1,'last');
%         
%         hist_mat_f(idx_f, idx_t) = hist_mat_f(idx_f, idx_t)+1;
%         hist_mat_v(idx_v, idx_t) = hist_mat_v(idx_v, idx_t)+1;
%     end
% 
% end







function fstr_label = get_fstr_label(fstr_res)

fstr_label = cell(size(fstr_res));

for i=1:numel(fstr_label)
    fstr_label{i} = "";
end

[~,b] = min(abs(fstr_res));
fstr_label{b} = "0";

[~,b] = min(abs(fstr_res-20));
fstr_label{b} = "20";

[~,b] = min(abs(fstr_res+20));
fstr_label{b} = "-20";

fstr_label = flip(fstr_label(2:end));
end


function vel_label = get_vel_label(vel_res)

vel_label = cell(size(vel_res));

for i=1:numel(vel_res)
    vel_label{i} = "";
end

[~,b] = min(abs(vel_res));
vel_label{b} = "0";

[~,b] = min(abs(vel_res-0.5));
vel_label{b} = "0.5";

[~,b] = min(abs(vel_res+0.5));
vel_label{b} = "-0.5";

vel_label = flip(vel_label(2:end));
end


function tlabel = get_tlabel(tres)

tlabel = cell(size(tres));

for i=1:numel(tres)
    tlabel{i} = "";
end

[~,b] = min(abs(tres));
tlabel{b} = "0";

[~,b] = min(abs(tres-1));
tlabel{b} = "1";

tlabel = tlabel(2:end);
end
