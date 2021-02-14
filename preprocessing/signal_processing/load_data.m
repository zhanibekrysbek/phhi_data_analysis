function [observations,tb] = load_data(base_path)
%load_data Summary of this function goes here
%   Detailed explanation goes here
    
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
            ind = ind+1;
        end
    end
    
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
end

