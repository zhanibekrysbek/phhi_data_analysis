function [] = plot_rfts(obs,opt)
%plot_rfts Plot Force Torque data
switch opt
    case 1
        subplot(221);
        plot(obs.rft1.time_steps, obs.rft1.force)
        subtitle(sprintf('%s Force', obs.rft1.frame_id))
        grid on;
        y1 = ylim;
        ylabel('N')
        xts = xticks;
        xticks(xts(1):xts(end));

        subplot(222);
        plot(obs.rft1.time_steps,obs.rft1.torque)
        subtitle(sprintf('%s Torque', obs.rft1.frame_id))
        grid on;
        y2 = ylim;
        xlabel('time, s')
        ylabel('Nm')
        xts = xticks;
        xticks(xts(1):xts(end));

        subplot(223);
        plot(obs.rft2.time_steps,obs.rft2.force)
        subtitle(sprintf('%s Force', obs.rft2.frame_id))
        grid on;
        y1 = [y1 ylim];
        xts = xticks;
        xticks(xts(1):xts(end));


        subplot(224);
        plot(obs.rft2.time_steps,obs.rft2.torque)
        subtitle(sprintf('%s Torque', obs.rft2.frame_id))
        grid on;
        y2 = [y2 ylim];
        xlabel('time, s')
        hL = legend({'x', 'y', 'z'},'Location','southwest','NumColumns',4);

        sgtitle(sprintf('%s %s %s RFT', obs.obs_id, obs.traj_type, obs.motion_type ), ...
            'Interpreter','none');


        subplot(221);
        ylim([min(y1) max(y1)])
        subplot(223);
        ylim([min(y1) max(y1)])

        subplot(222);
        ylim([min(y2), max(y2)])
        subplot(224);
        ylim([min(y2), max(y2)])
        
        
        set(hL, 'Position',[0.51 0.03 0.01 0.01],'Units','normalized')
        
    case 2
        subplot(321);
        plot(obs.rft1.time_steps, obs.rft1.forceS); hold on;
        plot(obs.rft1.time_steps, vecnorm(obs.rft1.forceS(:,1:2),2,2),'k--'); hold off;
        grid on;
        subtitle('Force 1');
        xts = xticks;
        xticks(xts(1):xts(end));
        ylabel('N');

        subplot(322);
        plot(obs.rft1.time_steps, obs.rft1.torqueS);
        grid on;
        subtitle('Torque 1 ');
        xts = xticks;
        xticks(xts(1):xts(end));
        ylabel('Nm');

        subplot(323);
        plot(obs.rft2.time_steps, obs.rft2.forceS); hold on;
        plot(obs.rft2.time_steps, vecnorm(obs.rft2.forceS(:,1:2),2,2),'k--'); hold off;
        grid on;
        subtitle('Force 2');
        hL = legend({'F_x', 'F_y', 'F_z', '||F_{xy}||'},'Location','southwest','NumColumns',4);
        xts = xticks;
        xticks(xts(1):xts(end));
        ylabel('N');


        subplot(324);
        plot(obs.rft2.time_steps, obs.rft2.torqueS);
        grid on;
        subtitle('Torque 2');
        xts = xticks;
        xticks(xts(1):xts(end));
        ylabel('Nm');
        

        subplot(325);
        plot(obs.rft1.time_steps, obs.fsum.forceS(:,1:2));hold on;
        plot(obs.rft1.time_steps, vecnorm(obs.fsum.forceS(:,1:2),2,2), 'k--'); hold off;
        grid on;
        subtitle('Force Sum');
        xts = xticks;
        xticks(xts(1):xts(end));
        ylabel('N');
        xlabel('sec');


        subplot(326);
        plot(obs.rft1.time_steps, obs.fstretch.forceS(:,1:2)); hold on;
        plot(obs.rft1.time_steps, vecnorm(obs.fstretch.forceS(:,1:2),2,2), 'k--'); hold off;
        grid on;
        subtitle('F stretch');
        xts = xticks;
        xticks(xts(1):xts(end));
        ylabel('N');
        xlabel('sec');
        
        sgtitle(sprintf('%s    %s    %s    Spatial Frame', obs.obs_id, obs.traj_type, obs.motion_type ), ...
            'Interpreter','none');
        set(hL, 'Position',[0.51 0.03 0.01 0.01],'Units','normalized')

end
    
end