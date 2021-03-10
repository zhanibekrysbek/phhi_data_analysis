


clc;clear;
base_path = '../../data/preprocessed_v2_2';

[observations_processed,tb] = load_data(base_path);

load('../../data/events/DetectedEvents_0.25sec.mat');

[features] = haptic_features(observations_processed);
% 
% st = struct2table(DetectedEvents);
% st = convertvars(st, {'obs_id'}, 'string');
% M = containers.Map(st.obs_id, st.events);



%% 

[primitives, primtable, primNum] = get_primitives(observations_processed, DetectedEvents);

btable.fstr_max = max(primtable.fstr_max);
btable.fstr_min = min(primtable.fstr_min);
btable.f1_max = max(primtable.f1_max);
btable.f1_min = min(primtable.f1_min);
btable.f2_max = max(primtable.f2_max);
btable.f2_min = min(primtable.f2_min);
btable.tau1_max = max(primtable.tau1_max);
btable.tau1_min = min(primtable.tau1_min);
btable.tau2_max = max(primtable.tau2_max);
btable.tau2_min = min(primtable.tau2_min);
btable.fsum_max = max(primtable.fsum_max);
btable.fsum_min = min(primtable.fsum_min);
btable.angs_max = max(primtable.angs_max);
btable.angs_min = min(primtable.angs_min);
btable.vel_max = max(primtable.vmax);
btable.vel_min = min(primtable.vmin);
btable.angvel_max = max(primtable.angvel_max);
btable.angvel_min = min(primtable.angvel_min);
btable.pos_max = max(primtable.pos_max);
btable.pos_min = min(primtable.pos_min);
btable.orient_max = max(primtable.orient_max);
btable.orient_min = min(primtable.orient_min);


primitives_norm = normalize_primitives(primitives, btable);

%% Distance

dist_mat = zeros(primNum,primNum);

for ind1 = progress(1:primNum, 'Title', 'PrimDistanceMatrix')
    prim1 = primitives_norm(ind1).prim;
    
    for ind2 = ind1:primNum
        prim2 = primitives_norm(ind2).prim;
        dist = prim_distance(prim1, prim2);
        dist_mat(ind1,ind2) = sum(dist);
    end
    
end

%%



%% Kmediods

dist_mat_sym = dist_mat'+dist_mat;

k = 7;
[inds,cidx] = kmedioids(dist_mat_sym,k);

cidx

%% DistMatrix

figure(35)
heatmap(dist_mat_sym(1:100,1:100));
figure(36)
histogram(inds)


%% Hiearchical

D = dist_mat';
D = D(:);
D = D(D~=0);

Z = linkage(D');
idx_norm_pca = cluster(Z,'maxclust',6);

cutoff = median([Z(end-2,3) Z(end-1,3)]);
dendrogram(Z,'ColorThreshold',cutoff)

%% Visualize Clusters
close all;
figure(40); 
for i = 1:k
    I = inds == i;
    scatter3(primtable.fstr_max(I,1), primtable.lfcons(I), primtable.duration(I)); hold on;
end
grid on;
hold off;



%%

prim1 = primitives_norm(1).prim;
prim2 = primitives_norm(2).prim;

dist = prim_distance(prim1,prim2);
