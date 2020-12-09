function [observations,tb] = load_data(base_path)
%load_data Summary of this function goes here
%   Detailed explanation goes here

    files = dir(base_path);

    observations(112) = struct();
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
            ind = ind+1;
        end
    end
    
    tb = struct2table(observations);
    tb = convertvars(tb, {'motion_type','traj_type', 'obs_id'}, 'string');
    try
        tb = removevars(tb, {'pose123', 'imu', 'rft1', 'rft2', 'fsum', 'fstretch'});
    catch
        tb = removevars(tb, {'pose123', 'imu', 'rft1', 'rft2'});
    end
end

