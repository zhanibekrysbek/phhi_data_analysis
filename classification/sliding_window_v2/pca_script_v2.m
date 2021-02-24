
clc;clear;close all;
base_path = '../../data/preprocessed_v2_2';

[observations_processed,tb] = load_data(base_path);

%% Customization
feature_inds = [0, 4, 7, 9, 12, 15, 18, 28];
feature_names = {'FT1', 'FT2', 'F_{str}', 'F_{sum}', 'pose', 'vel', '\alpha', 'F_{str}^{integ}'};
label_names = {'obs_id', 'starting_table', 'window_num', 'traj_type',...
    'motion_type', 'initialOrientation', 'outcomeSubject'};
label_maps = containers.Map(label_names, 1:numel(label_names));

data_version = {'body_frame', 'spatial_frame', 'body_n_haptics',...
    'spatial_n_haptics', 'haptics', 'body_frame_td', 'body_frame_td_tnorm'};
norm_option = {'unnormalized','normalized'};

%% Load Features

divisions = 1;
data_option = 7;

[X,Y] = extractSWFeatures(observations_processed, tb, data_option);
[N, numFeats] = size(X);

% Normalize
Xnorm = 2*(X-min(X))./(max(X)-min(X))-1;

% PCA
[coeff,score,latent] = pca(X);
[coeff_norm, score_norm, latent_norm] = pca(Xnorm);

X_pca = X*coeff;
Xnorm_pca = Xnorm*coeff_norm;

%% Box Plot of the data

I = 1:numFeats;
figure(31);
boxplot(X(:,I))
title('raw')
grid on;

figure(32);
boxplot(Xnorm(:,I))
title('normalized')
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

numF = 5;
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

        % Put vertical lines
        for j = 1:numel(feature_inds)
            xline(feature_inds(j)*divisions,'-.b', feature_names{j},....
                'LabelHorizontalAlignment', 'right', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal');
        end
%         xticks(sort([0:15:75, 95, 110, 125, 140:5:N]))
        xticks(feature_inds);

        ylim([-1., 1.]);
    end


    xlabel('features')
    ylabel('weights')
    sgtitle(sprintf('PCA weight dist %s %s', norm_option{tonorm}, data_version{data_option}), 'Interpreter','None');

    fig_path = ['../../data/plots/clustering/',data_version{data_option}];
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

    sgtitle(sprintf('PCA space %s %s', norm_option{tonorm}, data_version{data_option}), 'Interpreter','None');

    fig_path = ['../../data/plots/clustering/',data_version{data_option}];
%     exportgraphics(gcf, [fig_path, '/pca_space_',data_version{data_option},'_',norm_option{tonorm}, '.jpg'], 'Resolution', 300)

end



%% Visualize outcomeSubject/traj_type/initialOrientation/motion_type

idx_temp = Y(:, label_maps('traj_type'));

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

