function [X] = sliding_window(obs, wind_size, stride, divisions)
%sliding_window Summary of this function goes here
%   Detailed explanation goes here


% wind_size = 0.05;
% stride = 0.02;
% divisions = 5;

t0 = 0;
Nwinds = ceil((1 - wind_size)/stride);

X = zeros(Nwinds, divisions*25);

for ind = 1:Nwinds
    tf = t0 + wind_size;
    
    Irft = obs.rft1.tnorm <= tf & obs.rft1.tnorm >= t0;
    Ipos = obs.pose123.tnorm <= tf & obs.pose123.tnorm >= t0;
    Iimu = obs.imu.tnorm <= tf & obs.imu.tnorm >= t0;
    
    
    f1 = obs.rft1.force(Irft,:);
    f2 = obs.rft2.force(Irft,:);
    tor1 = obs.rft1.torque(Irft,:);
    tor2 = obs.rft2.torque(Irft,:);
    
    pos = obs.pose123.position(Ipos,:);
    orient = obs.pose123.orientation(Ipos,:);
    
    accel = obs.imu.accel(Iimu,:);
    gyro = obs.imu.gyro(Iimu,:);
    
    
    f1_v = extract_means(f1,divisions);
    f2_v = extract_means(f2,divisions);
    tor1_v = extract_means(tor1,divisions);
    tor2_v = extract_means(tor2,divisions);
    
    pos_v = extract_means(pos, divisions);
    orient_v = extract_means(orient , divisions);
    
    accel_v = extract_means(accel, divisions);
    gyro_v = extract_means(gyro, divisions);
    
    
    X(ind,:) = [f1_v tor1_v f2_v tor2_v pos_v orient_v accel_v gyro_v];
    
    
%     fprintf("%d %f\n",ind, t0);
    
    t0 = t0 + stride;
end


end


function mvec = extract_means(x,divisions)

    [len,dim] = size(x);
%     fprintf("%d %d\n", len, dim)
    
    sub_div = len/divisions;

    mvec = zeros(divisions,dim);
    for i=1:divisions
        t1 = floor((i-1)*sub_div)+1;
        t2 = floor(i*sub_div);
        mvec(i,:) = mean(x(t1:t2,:));
%         fprintf("%d %d\n", t1,t2);
    end

    mvec = reshape(mvec',1, numel(mvec));

end


