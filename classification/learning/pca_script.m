
clc;clear;
base_path = '../../data/preprocessed_v2_1';

[observations_processed,tb] = load_data(base_path);


%% Load Features

[X,Y] = load_slidwind(observations_processed);

%% Normalize

X = 2*(X-min(X))./(max(X)-min(X))-1;

%% PCA

[coeff,score,latent] = pca(X);


%% Visualize coeefs
close all;

numF = 3;
cmap = hsv(numF);
figure(1);
hold on;

for i=1:numF
scatter(1:140,coeff(i,:), 'filled', 'DisplayName', num2str(i), ... 
    'MarkerEdgeColor', cmap(i,:),'MarkerFaceColor', cmap(i,:)); 
p = plot(1:140,coeff(i,:), 'LineWidth', 0.1, 'color', cmap(i,:), 'DisplayName', '');
p.Color(4) = 0.2;
end
legend;
hold off;

xlabel('features')
ylabel('weights')
title('Weights of first few principle components')


%% Plot principal components

X_ = X*coeff;

figure(3);
subplot(1,2,1)
scatter3(X_(:,1),X_(:,2),X_(:,3)); grid on;
xlabel('1st component')
ylabel('2nd component')
zlabel('3rd component')

subplot(1,2,2)
scatter3(X_(:,4),X_(:,5),X_(:,6)); grid on;
xlabel('4th component')
ylabel('5th component')
zlabel('6th component')

sgtitle('First 3 Principal Components');





