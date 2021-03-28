function [observations,tb] = load_data(base_path)
%load_data Summary of this function goes here
%   Detailed explanation goes here
    
    if(exist(base_path)==0)
        error('Incorrect data path is provided');
    end

    global M
    
    subjectID = cell({'KOH'; 'Sanket'; 'Vignesh'; 'Zhanibek'});
    M = containers.Map(subjectID,[1:4]);
    
    Nobs = 112;
    files = dir(base_path);

    observations(Nobs) = struct();
    ind = 1;
    for i=progress(1:numel(files), 'Title', 'Loading')
        f = files(i);
        if ~f.isdir && endsWith(f.name, '.mat')
            ld = load(fullfile(f.folder,f.name));
            fns = fieldnames(ld);
            % Loop over field names
            for fn=1:numel(fns)
                observations(ind).(fns{fn}) = ld.(fns{fn});
            end
            observations(ind).duration = observations(ind).rft1.time_steps(end);
            observations(ind).outcomeSubject = getOutcomeSubject(observations(ind));
            observations(ind).pos_dec = getPosXdecision(observations(ind));
            observations(ind).tdec_sec = round(observations(ind).tdec_sec,2);
            observations(ind).tdec_sec = max(3, observations(ind).tdec_sec);
            
            ind = ind+1;
        end
    end
    
    observations(48).tdec_sec = observations(48).tdec_sec - 0.7;
    observations(48).pos_dec = getPosXdecision(observations(48));
    
    tb = struct2table(observations);
    tb = convertvars(tb, {'motion_type','traj_type', 'obs_id', ...
        'initialOrient', 'handle_1', 'handle_2', 'outcome'}, 'string');
    try
        tb = removevars(tb, {'pose123', 'imu', 'rft1', 'rft2', 'fsum', 'fstretch'});
    catch
        tb = removevars(tb, {'pose123', 'imu', 'rft1', 'rft2'});
    end
    tb.order = [1:height(tb)]';
    tb = [tb(:,end),tb(:,1:end-1)];
    
    % Fixing Typos in Annotations
    tb(tb.obs_id=='KOH_Sanket_16',:).initialOrient = 'xl';
    tb(tb.obs_id=='KOH_Sanket_9',:).initialOrient = 'xr';
    tb(tb.obs_id=='KOH_Sanket_7',:).initialOrient = 'xl';
    tb(tb.obs_id=='Sanket_Vignesh_1_3',:).initialOrient = 'xl';
    tb(tb.obs_id=='Sanket_Vignesh_1_9',:).tdec_sec = 3.5;
    
        
end


function  sub = getOutcomeSubject(obs)
    
    global M
    
    if contains(obs.outcome, '1')
        sub = M(obs.handle_1);
    else 
        sub = M(obs.handle_2);
    end
end


function pos = getPosXdecision(obs)
    
    I = obs.pose123.time_steps <= obs.tdec_sec;
    pos = obs.pose123.position(find(I,1,'last'),:);

end
