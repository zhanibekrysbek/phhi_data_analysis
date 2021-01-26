function [X,Y] = load_slidwind(observations_processed)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

wind_size = 0.05;
stride = 0.02;
divisions = 5;

Nwinds = ceil((1 - wind_size)/stride);

X = zeros(Nwinds*numel(observations_processed), divisions*28);
Y = zeros(Nwinds*numel(observations_processed), 5);

for ind=progress(1:numel(observations_processed), 'Title', 'ExtractingSWFeatures')
    obs = observations_processed(ind);
    [X((ind-1)*Nwinds+1:ind*Nwinds,:), Y((ind-1)*Nwinds+1:ind*Nwinds,3:end)] = ...
        sliding_window(obs, wind_size, stride, divisions);
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

