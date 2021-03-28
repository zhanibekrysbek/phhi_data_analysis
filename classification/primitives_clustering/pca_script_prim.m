
%% Customization
feature_inds = [0,30, 60, 95, 125];
feature_names = {'rft1', 'rft2', 'pose', 'twist', 'accel'};
label_names = {'obs_id', 'starting_table', 'window_num', 'traj_type',...
    'motion_type', 'initialOrientation', 'outcomeSubject'};
label_maps = containers.Map(label_names, 1:numel(label_names));

data_version = {'body_frame', 'spatial_frame', 'body_n_haptics', 'spatial_n_haptics', 'haptics', 'body_frame_td'};
norm_option = {'unnormalized','normalized'};

%% Cluster Primtable

% X = table2array(primtable(idx_fstr==3,[8:10, 19:27]));
X = table2array(primtable(:,[9:end]));
[N, numFeats] = size(X);

% Normalize
Xnorm = 2*(X-min(X))./(max(X)-min(X))-1;

% PCA
[coeff,score,latent] = pca(X);
[coeff_norm, score_norm, latent_norm] = pca(Xnorm);

X_pca = X*coeff;
Xnorm_pca = Xnorm*coeff_norm;
% Xnorm_pca = coeff_norm*Xnorm;

%% Box Plot of the data

I = 1:numFeats;
figure(31);
boxplot(X(:,I))
title('raw')
grid on;

figure(32);
boxplot(Xnorm(:,I))
title('raw')
grid on;

figure(33);
boxplot(Xnorm_pca(:,I))
title('PCA normalized')
grid on;

%% Information Content by Components

latent_temp = [0; latent_norm];

totVar = sum(latent_temp);
latent_temp = latent_temp/totVar;
latent_integ = zeros(size(latent_temp));
for i = 1:numel(latent_temp)
    latent_integ(i) = sum(latent_temp(1:i));
end

figure(41);
subplot(2,1,1)
scatter(0:size(X,2), latent_temp, 'filled', 'blue'); hold on;
p = plot(0:size(X,2), latent_temp, 'blue', 'LineWidth', 0.1);
subtitle('Variance by Components')
p.Color(4) = 0.2;
hold off; box on;
ylabel('variance')

subplot(2,1,2)
scatter(0:size(X,2), latent_integ, 'filled', 'red'); hold on;
p = plot(0:size(X,2), latent_integ, 'red', 'LineWidth', 0.1);
subtitle('Variance accumulated')
p.Color(4) = 0.2;
hold off; box on;
xlabel('Principal Components')
ylabel('Accumulated variance')

%% Visualize coeffs
% close all;

numF = 4;
cmap = hsv(numF);


for tonorm = 1:2
    figure(tonorm);
    
    if tonorm==1
        C = coeff;
    else
        C = coeff_norm;
    end
    
    for i=1:numF
        subplot(numF,1,i)

        scatter(1:numFeats,C(i,:), 'filled', 'DisplayName', num2str(i), ... 
            'MarkerEdgeColor', cmap(i,:),'MarkerFaceColor', cmap(i,:)); 
        hold on;
        p = plot(1:numFeats,C(i,:), 'LineWidth', 0.1, 'color', cmap(i,:), 'DisplayName', '');
        p.Color(4) = 0.2;
        hold off; grid on; box on;

        subtitle(sprintf('Principal Component %d', i))

%         % Put vertical lines
%         for j = 1:numel(feature_inds)
%             xline(feature_inds(j),'-.b', feature_names{j},....
%                 'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal');
%         end
%         xticks(sort([0:15:75, 95, 110, 125, 140:5:N]))

        ylim([-1., 1.]);
    end


    xlabel('features')
    ylabel('weights')
    sgtitle(sprintf('PCA weight dist %s', norm_option{tonorm}), 'Interpreter','None');

%     fig_path = ['../../data/plots/clustering/',data_version{data_option}];
%     exportgraphics(gcf, [fig_path, '/pca_weights_',data_version{data_option},'_',norm_option{tonorm}, '.jpg'], 'Resolution', 300)
end
%% Plot principal components

for tonorm = 1:2
    
    figure(tonorm+5);
    
    if tonorm==1
        Xtemp = X_pca;
    else
        Xtemp = Xnorm_pca;
    end
    
    scatter3(Xtemp(:,1),Xtemp(:,2),Xtemp(:,3)); grid on;
    xlabel('1st component')
    ylabel('2nd component')
    zlabel('3rd component')

%     sgtitle(sprintf('PCA space %s %s', norm_option{tonorm}, data_version{data_option}), 'Interpreter','None');

%     fig_path = ['../../data/plots/clustering/',data_version{data_option}];
%     exportgraphics(gcf, [fig_path, '/pca_space_',data_version{data_option},'_',norm_option{tonorm}, '.jpg'], 'Resolution', 300)

end



%% Visualize outcomeSubject/traj_type/initialOrientation/motion_type

idx_temp = (primtable.outcome+1)/2 + 1;

k = numel(unique(idx_temp));

Xtemp = Xnorm_pca;
figure(13);
cmap = hsv(k);

obs_id = 100;
c = zeros(numel(idx_temp),3);
for i =  1:k % numel(idx_temp)
    c(i,:) = cmap(idx_temp(i),:);
    Is = idx_temp==i;
    scatter3(Xtemp(Is,1), Xtemp(Is,2), Xtemp(Is,3),10,cmap(i,:)); hold on;
end

xlabel('Component 1')
ylabel('Component 2')
zlabel('Component 3')

grid on; hold off;
legend(num2str([1:k]'));


%% Kmeans


k = 3;
ncomponents = 20;
[idx_norm_pca,C_norm_pca,sumd_norm,D_norm] = kmeans(Xnorm_pca(:,1:ncomponents),k);

idx_temp = idx_norm_pca;
Xtemp = Xnorm_pca;


figure(36)
histogram(idx_temp)


%% 3D plot in PCA space

idx_temp = idx_norm_pca;
Xtemp = Xnorm_pca;
figure(13);
cmap = hsv(k);

obs_id = 100;
c = zeros(numel(idx_temp),3);
for i =  1:k%numel(idx_temp)
    c(i,:) = cmap(idx_temp(i),:);
    Is = idx_temp==i;
    scatter3(Xtemp(Is,1), Xtemp(Is,2), Xtemp(Is,3),30,cmap(i,:), 'filled'); hold on;
end

xlabel('Component 1')
ylabel('Component 2')
zlabel('Component 3')

grid on; hold off;
legend(num2str([1:k]'));


%% Cluster visualization


figure(77);

% numF = 4;
cmap = hsv(k);
% varnames = {'fstr_max', 'fstr_min', 'outcome', 'fsum_max', 'exp_dir', ...
%         'inst_dec', 'lfcons', 'seqnum'};

varnames = {'fstr_max', 'fstr_min', 'vx_gain', 'vy_gain', ...
        'pos_x_mean', 'pos_y_mean', 'duration', 'fstr_t_extr', 'fsumx_max',  'lfcons', ...
        'exp_dir', 'inst_dec', 'seqnum', 'outcome'};


for i=1:k
    
    cl = primtable(idx_temp==i,:);
    
%     clfeat = table2array(cl(:,varnames));
    clfeat = [ cl.fstr_max(:,1)  cl.fstr_min(:,1)...
        20*cl.v_gain, 20*cl.pos_mean, 20*cl.duration-20, 20*cl.fstr_t_extr-15, ...
        3*cl.fsum_max(:,1) 10*cl.seqnum-15 10*cl.exp_dir 20*cl.inst_dec-10 ...
        20*cl.lfcons-30  10*cl.outcome];
    
    numF = size(clfeat,2);
    
    N = size(clfeat,1);                                      % Number of ‘Experiments’ In Data Set
    yMean = mean(clfeat);                                    % Mean Of All Experiments At Each Value Of ‘x’
    ySEM = std(clfeat)/sqrt(N);                              % Compute ‘Standard Error Of The Mean’ Of All Experiments At Each Value Of ‘x’
    CI95 = tinv([0.025 0.975], N-1);                         % Calculate 95% Probability Intervals Of t-Distribution
    yCI95 = bsxfun(@times, ySEM, CI95(:));   
    
%     [ median(cl.fstr_max(:,1)) mean(cl.fstr_min(:,1)) 10*mean(cl.outcome) ...
%         mean(cl.fsum_max) 5*mean(cl.exp_dir) 10*mean(cl.inst_dec)-5 ...
%         10*mean(cl.lfcons)-15 2*mean(cl.seqnum)];
    
%     scatter(1:numel(clfeat), clfeat, 'filled', 'DisplayName', num2str(i), ... 
%         'MarkerEdgeColor', cmap(i,:),'MarkerFaceColor', cmap(i,:)); hold on;

%     p = plot(1:numel(clfeat), clfeat, 'LineWidth', 0.1, 'color', cmap(i,:), 'DisplayName', '');
%         p.Color(4) = 0.2;
    
    sh = (i-1)*0.1;
    xax = [1:numF] + sh;
    
    scatter(xax, yMean, 'filled', 'DisplayName', num2str(i), ... 
        'MarkerEdgeColor', cmap(i,:),'MarkerFaceColor', cmap(i,:)); hold on;
    
    scatter(xax, yCI95(1,:)+yMean,'^', 'DisplayName', num2str(i), ... 
        'MarkerEdgeColor', cmap(i,:),'MarkerFaceColor', cmap(i,:));
    
    scatter(xax, yCI95(2,:)+yMean,'v', 'DisplayName', num2str(i), ... 
        'MarkerEdgeColor', cmap(i,:),'MarkerFaceColor', cmap(i,:));
    
    
    p = plot(xax, yCI95(1,:)+yMean, 'LineWidth', 0.1, 'color', cmap(i,:), 'DisplayName', '');
    p.Color(4) = 0.2;
    p = plot(xax, yCI95(2,:)+yMean, 'LineWidth', 0.1, 'color', cmap(i,:), 'DisplayName', '');
    p.Color(4) = 0.2;
    p = plot(xax, yMean, 'LineWidth', 0.1, 'color', cmap(i,:), 'DisplayName', '');
    p.Color(4) = 0.2;
    
    
    
    
end

% ax = gca;
% ax.XAxis.XTickLable = varnames;
% ax.XAxis.TickLabelRotation = 30;

% xticks(1:numF);
set(gca,'xtick',[1:numF],'xticklabel',varnames,...
    'TickLabelInterpreter','None', 'XTickLabelRotation',30);


hold off; grid on; box on;

% legend(num2str([1:k]'));
legend;

xlabel('features')
sgtitle(sprintf('Cluster (k=%d) Visualization with 95%% Conf Interval', k), 'Interpreter','None');


%% Plot Fstr-vel Shapes

ks = unique(primtable.cluster);

for j = 1:numel(ks)
    
    sl = primtable(primtable.cluster==ks(j),:);
    figure(300+j); 
    subplot(2,2,1);
    for i=1:size(sl,1)
       prim = sl.prim(i); 
       t = prim.time_steps - prim.time_steps(1);
       t = t/t(end);
       
       if sl.exec_prim(i)
            plot(t, prim.fstr(1:numel(t)),'r', 'LineWidth', 0.5);
            hold on;
            continue;
       end
       
       plot(t, prim.fstr(1:numel(t)),'b', 'LineWidth', 0.5);
       hold on;
       
    end
    
    ylim([-20, 40])
    hold off; grid on;
    ylabel('F_{str}')
    subtitle('F_{str}');
    
    
%     figure(400+j);
    subplot(2,2,2);
    for i=1:size(sl,1)
       prim = sl.prim(i); 
       t = prim.time_steps - prim.time_steps(1);
       t = t/t(end);

        if sl.exec_prim(i)
            plot(t, prim.vel(1:numel(t),2),'r', 'LineWidth', 0.5);
            hold on;
            continue;
        end
       
       plot(t, (prim.vel(1:numel(t),2)),'b', 'LineWidth', 0.5);
       hold on;
       
    end
    ylim([-0.7, 0.7])
    ylabel('v_{y}'); grid on;
    subtitle('v_{y}');
    sgtitle(sprintf('cluster: %d #points: %d', ks(j), size(sl,1)));
    hold off;
    
    [hist_mat_f,hist_mat_v] = signal_histogram(sl);
    
    subplot(2,2,3);
    heatmap(hist_mat_f, 'Colormap', jet);
    
    subplot(2,2,4);
    heatmap(hist_mat_v, 'Colormap', jet);
    
end

%% Visualize dist_mat heatmaps

dist_mat_temp = dist_mat_fv;

dist_mat_temp([196],:) = []; 
dist_mat_temp(:,[196]) = [];

idx_temp1 = idx_temp;
idx_temp1([196]) = [];

[B,I] = sort(idx_temp1);

figure(499)
heatmap(dist_mat_temp(I,I), 'Colormap', turbo);







