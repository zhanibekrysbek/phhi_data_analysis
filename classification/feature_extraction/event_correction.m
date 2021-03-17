function [DetectedEvents] = event_correction(DetectedEvents, observations_processed)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

fname = "../../data/annotation/annotation - event_correction.csv";

st = struct2table(observations_processed);
st = convertvars(st, {'obs_id'}, 'string');

evCor = csvread(fname);

for ev_ind = 1:numel(DetectedEvents)

    obs_ind = find(st.obs_id == DetectedEvents(ev_ind).obs_id);
    
    cor_mat = evCor(evCor(:,1)==obs_ind,2:3);
    
    if ~isempty(cor_mat)
        DetectedEvents(ev_ind).events = cor_mat;
    end
    
end

end
