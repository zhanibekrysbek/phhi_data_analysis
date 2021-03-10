function plot_events(obs, feat, win_locs, opt)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


switch opt
    
    case 1
        
        I = feat.angles.time_steps >= win_locs(1,1) & feat.angles.time_steps <= obs.tdec_sec;
        tang = feat.angles.time_steps(I);
        fstr = obs.fstretch.force(1:10:end,1:2); fstr = fstr(I,:);
        fsum = obs.fsum.forceS(1:10:end,:); fsum = fsum(I,:);
        f1 = obs.rft1.forceS(1:10:end,1:2); f1 = f1(I,:);
        f2 = obs.rft2.forceS(1:10:end,1:2); f2 = f2(I,:);
        tau1 = obs.rft1.ttorqueS(1:10:end,3); tau1 = tau1(I,:);
        tau2 = obs.rft2.ttorqueS(1:10:end,3); tau2 = tau2(I,:);
        
        orient = rad2deg(obs.imu.orientation(I,4));
        orient = orient - orient(1);
        
        tmax = max(5,obs.tdec_sec + 0.5);
        
        srows = 4;
        scols = 2;

        % First Column
        subplot(srows, scols, 1);
        a1 = plot(tang, feat.angles.angles(I,2),'b'); hold on; 
        a2 = plot(tang, feat.angles.angles(I,3), 'm'); grid on; 
        ylim([0 180]);
        xlim([0, tmax]);
        subtitle('F_1 vs v and F_2 vs v angles');
        plot_vertical_lines(win_locs);
        hL = legend([a1,a2],{'x', 'y'},'NumColumns',4);
        hold off;

        subplot(srows, scols, 3);
        plot(tang, fstr(:,1), 'b', tang, fstr(:,2),'m'); grid on; hold on;
        subtitle('F_{str} body'); hold off;
        ymax = max(20,max(abs(fstr(:))+0.4));
        ylim([-ymax, ymax]);
%         ylim([-20,20]);
        xlim([0, tmax]);
       	plot_vertical_lines(win_locs);

        subplot(srows, scols, 5);
        plot(tang, obs.pose123.linvel(I,1), 'b', tang, obs.pose123.linvel(I,2), 'm'); grid on;
        subtitle('v_x v_y');
        ymax = max(0.5, max(max(abs(obs.pose123.linvel(I,:)))));
        ylim([-ymax ymax]);
        xlim([0, tmax]);
        plot_vertical_lines(win_locs);
        
        subplot(srows, scols, 7);
        plot(tang, orient); grid on;
        subtitle('orientation angle');
        xlim([0, tmax]);
        ymax = max(20, max(abs(orient)));
        ylim([-ymax, ymax]);
        xlabel('time, sec');
        plot_vertical_lines(win_locs);
        hold off;
        xlabel('time, sec');
        
        
        % Second Column
        subplot(srows, scols, 2);
        a1 = plot(tang, f1(:,1),'b', tang, f1(:,2), 'm'); hold on; 
        a2 = plot(tang, f2(:,1),'b--', tang, f2(:,2), 'm--'); grid on; 
        xlim([0, tmax]);
        ymax = max(20,max(abs(fstr(:))+0.4));
        ylim([-ymax, ymax]);
        subtitle('F_1 and F_2 spatial');
        plot_vertical_lines(win_locs);
%         legend([a1',a2'],{'f_{1x}', 'f_{1y}', 'f_{2x}', 'f_{2y}'});
        hold off;
        
        
        subplot(srows, scols, 4);
        a1 = plot(tang, fsum(:,1),'b', tang, fsum(:,2), 'm'); hold on; grid on; 
%         legend([a1'],{'f_{x}^{sum}', 'f_{y}^{sum}'});
        xlim([0, tmax]);
%         ylim([-12, 12]);
        ylim([-20, 20]);
        subtitle('F_{sum} spatial');
        plot_vertical_lines(win_locs);
        hold off;
        
        
        subplot(srows, scols, 6);
        a1 = plot(tang, tau1,'b'); hold on; 
        a2 = plot(tang, tau2,'m'); grid on; 
%         legend([a1',a2'],{'\tau_{1z}', '\tau_{2z}'});
        xlim([0, tmax]);
        ylim([-2, 2]);
        subtitle('\tau_1 and \tau_2 spatial');
        plot_vertical_lines(win_locs);
        hold off;
        
        
        subplot(srows, scols, 8);
        a1 = plot(tang, rad2deg(obs.imu.angvel(I,3))); hold on; grid on; 
%         legend([a1'],{'\omega_z'});
        xlim([0, tmax]);
        ylim([-70, 70]);
        subtitle('\omega_z');
        plot_vertical_lines(win_locs);
        hold off;
        xlabel('time, sec')
        
        set(hL, 'Position',[0.5 0.03 0.01 0.01],'Units','normalized')        
        sgtitle(sprintf('%s %s %s RFT Mix Plot', obs.obs_id, obs.traj_type, obs.motion_type ), ...
            'Interpreter','none');
        
        
    case 2

    I = feat.angles.time_steps >= win_locs(1,1) & feat.angles.time_steps <= obs.tdec_sec;
    tang = feat.angles.time_steps(I);
    ang = feat.angles.angles(I,:);



    tmax = max(5,obs.tdec_sec + 0.5);

    srows = 3;
    scols = 2;


    % First Column
    
    subplot(srows, scols, 1);
    a1 = plot(tang, feat.angles.angles(I,1),'b'); hold on; 
    grid on; 
    ylim([0 180]);
    xlim([0, tmax]);
    subtitle('F_1 vs F_2');
    plot_vertical_lines(win_locs);
    
    hold off;
    
    
    
    subplot(srows, scols, 3);
    a1 = plot(tang, feat.angles.angles(I,2),'b'); hold on; 
    a2 = plot(tang, feat.angles.angles(I,3), 'm'); grid on; 
    ylim([0 180]);
    xlim([0, tmax]);
    subtitle('F_1 vs v and F_2 vs v');
    plot_vertical_lines(win_locs);
    hL = legend([a1,a2],{'x', 'y'},'NumColumns',4);
    hold off;
    
    
    subplot(srows, scols, 5);
    a1 = plot(tang, feat.angles.angles(I,6),'b'); hold on; 
    a2 = plot(tang, feat.angles.angles(I,7), 'm'); grid on; 
    ylim([0 180]);
    xlim([0, tmax]);
%     subtitle('F_1 vs F_{sum} and F_2 vs F_{sum}');
    plot_vertical_lines(win_locs);
%     hL = legend([a1,a2],{'x', 'y'},'NumColumns',4);
    hold off;
    
    
    
    subplot(srows, scols, 2);
    a1 = plot(tang, feat.angles.angles(I,9),'b'); hold on; 
    a2 = plot(tang, feat.angles.angles(I,10), 'm'); grid on; 
    ylim([0 180]);
    xlim([0, tmax]);
    subtitle('F_1 vs F_{str} and F_2 vs F_{str}');
    plot_vertical_lines(win_locs);
%     hL = legend([a1,a2],{'x', 'y'},'NumColumns',4);
    hold off;
    
    
    subplot(srows, scols, 4);
    a1 = plot(tang, feat.angles.angles(I,4),'b'); hold on; 
    a2 = plot(tang, feat.angles.angles(I,5), 'm'); grid on; 
    ylim([0 180]);
    xlim([0, tmax]);
    subtitle('F_{sum} vs v and F_{str} vs v');
    plot_vertical_lines(win_locs);
%     legend([a1, a2], {'F_{sum} v', 'F_{str} v'})
    hold off;
    
    
    subplot(srows, scols, 6);
    a1 = plot(tang, feat.angles.angles(I,8),'b'); hold on; grid on;
    ylim([0 180]);
    xlim([0, tmax]);
    subtitle('F_{sum} vs F_{str}');
    plot_vertical_lines(win_locs);
%     legend([a1, a2], {'F_{sum} v', 'F_{str} v'})
    hold off;
    
    
    xlabel('time, sec')
        
    set(hL, 'Position',[0.5 0.03 0.01 0.01],'Units','normalized')        
    sgtitle(sprintf('%s %s %s RFT Angles', obs.obs_id, obs.traj_type, obs.motion_type ), ...
        'Interpreter','none');

end



end


function plot_vertical_lines(win_locs)

    for i = 1:size(win_locs,1)
            a = xline(win_locs(i,1),'-.k', 't0');
            a = xline(win_locs(i,2),'-.k', 'tf','LabelVerticalAlignment','bottom');
    end
end

