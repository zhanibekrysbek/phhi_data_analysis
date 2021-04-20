function [Stable, S, Snorm] = transition_mat(tb_temp)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
nclusters = numel(unique(tb_temp.cluster));

S = zeros([nclusters+1,nclusters+1]);

% loop over trajectoriesk
for ind = unique(tb_temp.obs_ind)'
   
    seq = tb_temp.cluster(tb_temp.obs_ind==ind);
    
    S(end, seq(1)) = S(end, seq(1))+1;
    
    if numel(seq)==1
        S(seq, nclusters+1) = S(seq, nclusters+1) + 1;
        continue;
    end
    
    % loop over sequences
    for ind1 = 1:numel(seq)-1
        S(seq(ind1), seq(ind1+1)) = S(seq(ind1), seq(ind1+1))+1;
    end
    S(seq(ind1+1), nclusters+1) = S(seq(ind1+1), nclusters+1)+1;
end

Snorm = S./sum(S,2);

rownames = cell([nclusters+1, 1]);
colnames = cell([nclusters+1, 1]);
for i = 1:numel(rownames)
    rownames{i} = ['C' num2str(i)];
    colnames{i} = ['C' num2str(i)];
end
colnames{end} = 'term';
rownames{end} = 'start';

Stable = array2table(Snorm,  'VariableNames', ...
    colnames, 'RowNames',...
    rownames);


end

