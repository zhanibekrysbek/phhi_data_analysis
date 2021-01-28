function [] = plot_rfts(obs,opt)
switch opt
    case 1
        % Forces in Body Frame
        subplot(321);
        plot(obs.rft1.time_steps,obs.rft1.force); hold on;
        plot(obs.rft1.time_steps, vecnorm(obs.rft1.force(:,1:2),2,2),'k--'); hold off;
        hL = legend({'x', 'y', 'z', '||xy||'},'Location','southwest','NumColumns',4);


        subtitle(sprintf('%s Force', obs.rft1.frame_id))
        grid on;
        y1 = ylim;
        ylabel('N')

        subplot(322);
        plot(obs.rft1.time_steps,obs.rft1.torque)
        subtitle(sprintf('%s Torque', obs.rft1.frame_id))
        grid on;
        y2 = ylim;
        xlabel('time, s')
        ylabel('Nm')

        subplot(323);
        plot(obs.rft2.time_steps,obs.rft2.force); hold on;
        plot(obs.rft2.time_steps, vecnorm(obs.rft2.force(:,1:2),2,2),'k--'); hold off;
        subtitle(sprintf('%s Force', obs.rft2.frame_id))
        grid on;
        y1 = [y1 ylim];


        subplot(324);
        plot(obs.rft2.time_steps,obs.rft2.torque)
        subtitle(sprintf('%s Torque', obs.rft2.frame_id))
        grid on;
        y2 = [y2 ylim];
        xlabel('time, s')


        subplot(325);
        plot(obs.rft1.time_steps, obs.fsum.force(:,1:2)); hold on;
        plot(obs.rft1.time_steps, vecnorm(obs.fsum.force(:,1:2),2,2), 'k--'); hold off;
        grid on;
        subtitle('Force Sum');


        subplot(326);
        plot(obs.rft1.time_steps, obs.fstretch.force(:,1:2)); hold on;
        plot(obs.rft1.time_steps, vecnorm(obs.fstretch.force(:,1:2),2,2), 'k--'); hold off
        grid on;
        subtitle('F stretch');


        sgtitle(sprintf('%s %s %s RFT Body Frame', obs.obs_id, obs.traj_type, obs.motion_type ), ...
            'Interpreter','none');

        set(hL, 'Position',[0.5 0.03 0.01 0.01],'Units','normalized')

        subplot(321);
        ylim([min(y1) max(y1)])
        subplot(323);
        ylim([min(y1) max(y1)])

        subplot(322);
        ylim([min(y2), max(y2)])
        subplot(324);
        ylim([min(y2), max(y2)])



    case 2

        % Forces are in Spatial Frame
        subplot(321);
        plot(obs.rft1.time_steps, obs.rft1.forceS); hold on;
        plot(obs.rft1.time_steps, vecnorm(obs.rft1.forceS(:,1:2),2,2),'k--'); hold off;
        hL = legend({'x', 'y', 'z', '||xy||'},'Location','southwest','NumColumns',4);


        grid on;
        subtitle('Force 1 ');

        subplot(322);
        plot(obs.rft1.time_steps, obs.rft1.torqueS);
        grid on;
        subtitle('Torque 1 ');

        subplot(323);
        plot(obs.rft2.time_steps, obs.rft2.forceS); hold on;
        plot(obs.rft2.time_steps, vecnorm(obs.rft2.forceS(:,1:2),2,2),'k--'); hold off;
        grid on;
        subtitle('Force 2 ');


        subplot(324);
        plot(obs.rft2.time_steps, obs.rft2.torqueS);
        grid on;
        subtitle('Torque 2');

        subplot(325);
        plot(obs.rft1.time_steps, obs.fsum.forceS(:,1:2)); hold on;
        plot(obs.rft1.time_steps, vecnorm(obs.fsum.forceS(:,1:2),2,2), 'k--'); hold off;
        grid on;
        subtitle('Force Sum');


        subplot(326);
        plot(obs.rft1.time_steps, obs.fstretch.forceS(:,1:2)); hold on;
        plot(obs.rft1.time_steps, vecnorm(obs.fstretch.forceS(:,1:2),2,2), 'k--'); hold off
        grid on;
        subtitle('F stretch');


        sgtitle(sprintf('%s %s %s RFT Spatial Frame', obs.obs_id, obs.traj_type, obs.motion_type ), ...
            'Interpreter','none');


        set(hL, 'Position',[0.5 0.03 0.01 0.01],'Units','normalized')
end
    
end