function [] = plot_rfts(obs,opt)
switch opt
    case 1
        %% Forces in Body Frame
        subplot(321);
        plot(obs.rft1.time_steps,obs.rft1.force); hold on;
%         plot(obs.rft1.time_steps, vecnorm(obs.rft1.force(:,1:2),2,2),'k--');
        hold off;
        hL = legend({'x', 'y', 'z'},'Location','southwest','NumColumns',4);
        xline(obs.tdec_sec,'-.b', 'td',...
                'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')


        subtitle(sprintf('%s Force', obs.rft1.frame_id))
        grid on;
        y1 = ylim;
        ylabel('N')

        subplot(322);
        plot(obs.rft1.time_steps,obs.rft1.torque)
        subtitle(sprintf('%s Torque', obs.rft1.frame_id))
        xline(obs.tdec_sec,'-.b', 'td',...
                'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')

        grid on;
        y2 = ylim;
        xlabel('time, s')
        ylabel('Nm')

        subplot(323);
        plot(obs.rft2.time_steps,obs.rft2.force); hold on;
%         plot(obs.rft2.time_steps, vecnorm(obs.rft2.force(:,1:2),2,2),'k--'); 
        hold off;
        subtitle(sprintf('%s Force', obs.rft2.frame_id))
        xline(obs.tdec_sec,'-.b', 'td',...
                'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')

        grid on;
        y1 = [y1 ylim];


        subplot(324);
        plot(obs.rft2.time_steps,obs.rft2.torque)
        subtitle(sprintf('%s Torque', obs.rft2.frame_id))
        xline(obs.tdec_sec,'-.b', 'td',...
                'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')

        grid on;
        y2 = [y2 ylim];
        xlabel('time, s')


        subplot(325);
        plot(obs.rft1.time_steps, obs.fsum.force(:,1:2)); hold on;
%         plot(obs.rft1.time_steps, vecnorm(obs.fsum.force(:,1:2),2,2), 'k--'); 
        hold off;
        xline(obs.tdec_sec,'-.b', 'td',...
                'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')

        grid on;
        subtitle('Force Sum');


        subplot(326);
        plot(obs.rft1.time_steps, obs.fstretch.force(:,1:2)); hold on;
%         plot(obs.rft1.time_steps, vecnorm(obs.fstretch.force(:,1:2),2,2), 'k--');
        hold off
        xline(obs.tdec_sec,'-.b', 'td',...
                'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')

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
         %% Forces are in Spatial Frame
        subplot(321);
        plot(obs.rft1.time_steps, obs.rft1.forceS); hold on;
%         plot(obs.rft1.time_steps, vecnorm(obs.rft1.forceS(:,1:2),2,2),'k--'); 
        hold off;
        hL = legend({'x', 'y', 'z'},'Location','southwest','NumColumns',4);
        xline(obs.tdec_sec,'-.b', 'td',...
                'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')


        grid on;
        subtitle('Force 1 ');

        subplot(322);
        plot(obs.rft1.time_steps, obs.rft1.torqueS);
        xline(obs.tdec_sec,'-.b', 'td',....
                'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')

        grid on;
        subtitle('Torque 1 ');

        subplot(323);
        plot(obs.rft2.time_steps, obs.rft2.forceS); hold on;
%         plot(obs.rft2.time_steps, vecnorm(obs.rft2.forceS(:,1:2),2,2),'k--'); 
        hold off;
        xline(obs.tdec_sec,'-.b', 'td',...
                'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')

        grid on;
        subtitle('Force 2 ');


        subplot(324);
        plot(obs.rft2.time_steps, obs.rft2.torqueS);
        xline(obs.tdec_sec,'-.b', 'td',....
                'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')

        grid on;
        subtitle('Torque 2');

        subplot(325);
        plot(obs.rft1.time_steps, obs.fsum.forceS(:,1:2)); hold on;
%         plot(obs.rft1.time_steps, vecnorm(obs.fsum.forceS(:,1:2),2,2), 'k--'); 
        hold off;
        xline(obs.tdec_sec,'-.b', 'td',...
                'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')

        grid on;
        subtitle('Force Sum');


        subplot(326);
        plot(obs.rft1.time_steps, obs.fstretch.forceS(:,1:2)); hold on;
%         plot(obs.rft1.time_steps, vecnorm(obs.fstretch.forceS(:,1:2),2,2), 'k--');
        hold off
        xline(obs.tdec_sec,'-.b', 'td',...
                'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')

        grid on;
        subtitle('F stretch');


        sgtitle(sprintf('%s %s %s RFT Spatial Frame', obs.obs_id, obs.traj_type, obs.motion_type ), ...
            'Interpreter','none');


        set(hL, 'Position',[0.5 0.03 0.01 0.01],'Units','normalized')



        % Haptics

    
    case 3
        %% Wrench SE3
        snr = 4;
        snc = 2;
        
        % Force
        subplot(snr,snc,1)
        plot(obs.rft1.time_steps, obs.rft1.force)
        grid on; title('F_1 [N]');
        xline(obs.tdec_sec,'-.b', 'td',...
                        'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')
        
        subplot(snr,snc,3)
        plot(obs.rft2.time_steps, obs.rft2.force)
        grid on; title('F_2')
        xline(obs.tdec_sec,'-.b', 'td',...
                        'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')
        
        
        subplot(snr,snc,5)
        plot(obs.rft1.time_steps, obs.rft1.force + obs.rft2.force)
        grid on;title('F_{sum}')
        xline(obs.tdec_sec,'-.b', 'td',...
                        'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')
        
        
        subplot(snr,snc,7)
        plot(obs.rft1.time_steps, obs.rft1.force - obs.rft2.force)
        grid on; title('F_{str}')
        xline(obs.tdec_sec,'-.b', 'td',...
                        'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')
        xlabel('time, [sec]')
        
        
        % Torque
        subplot(snr,snc,2)
        plot(obs.rft2.time_steps, obs.rft1.ttorque)
        grid on; title('\tau_1^{tot} [Nm]')
        xline(obs.tdec_sec,'-.b', 'td',...
                        'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')
        
        
        subplot(snr,snc,4)
        plot(obs.rft2.time_steps, obs.rft2.ttorque)
        grid on; title('\tau_2^{tot}')
        xline(obs.tdec_sec,'-.b', 'td',...
                        'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')
        
        
        subplot(snr,snc,6)
        plot(obs.rft1.time_steps, obs.rft1.ttorque + obs.rft2.ttorque)
        grid on;title('\tau_{sum}')
        xline(obs.tdec_sec,'-.b', 'td',...
                        'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')
        
        
        subplot(snr,snc,8)
        plot(obs.rft1.time_steps, obs.rft1.ttorque - obs.rft2.ttorque)
        grid on; title('\tau_{str}');
        xline(obs.tdec_sec,'-.b', 'td',...
                        'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')
        xlabel('time, [sec]')
        
        
        hL = legend({'x', 'y', 'z'}, 'Location','southwest','NumColumns',3);
        
        
        sgtitle(sprintf('%s %s %s RFT BF SE3', obs.obs_id, obs.traj_type, obs.motion_type ), ...
                    'Interpreter','none');
        
        set(hL, 'Position',[0.5 0.03 0.01 0.01],'Units','normalized')


    case 4
        %% Wrench + Kinematics in SE2

        snr = 4;
        snc = 2;
        
        % #0072BD #D95319 #EDB120
        
        % Force
        subplot(snr,snc,1)
        
        plot(obs.rft1.time_steps, obs.rft1.force(:,[1,2]), 'LineWidth', 1, 'LineStyle', '-'); hold on;
        grid on; title('F_1 [N]');
        xline(obs.tdec_sec,'-.b', 'td',...
                        'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')
        legend({'x','y'}, 'NumColumns', 2, 'Location', 'best')
        ylabel('N');
        ytemp = ylim; ylims_force = [ytemp];
        hold off;
        
        
        subplot(snr,snc,3)
        plot(obs.rft2.time_steps, obs.rft2.force(:,[1,2]), 'LineWidth', 1)
        grid on; title('F_2')
        xline(obs.tdec_sec,'-.b', 'td',...
                        'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal');
        hold off;
        legend({'x','y'}, 'NumColumns', 2, 'Location', 'best')
        ylabel('N');
        ytemp = ylim; ylims_force = [ytemp];
        
        
        
        subplot(snr,snc,5)
        plot(obs.rft1.time_steps, obs.rft1.force(:,[1,2]) + obs.rft2.force(:,[1,2]), 'LineWidth', 1)
        grid on;title('F_{sum}')
        xline(obs.tdec_sec,'-.b', 'td',...
                        'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')
        legend({'x','y'}, 'NumColumns', 2, 'Location', 'best')
        ytemp = ylim; ylims_force = [ytemp ylims_force];
        ylabel('N')
        
        
        subplot(snr,snc,7)
        plot(obs.rft1.time_steps, obs.rft1.force(:,[1,2]) - obs.rft2.force(:,[1,2]), 'LineWidth', 1)
        grid on; title('F_{str}')
        xline(obs.tdec_sec,'-.b', 'td',...
                        'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')
        xlabel('time, [sec]')
        legend({'x','y'}, 'NumColumns', 2, 'Location', 'best')
        ytemp = ylim; ylims_force = [ytemp ylims_force];
        ylabel('N')
        
        
        % Torque
        subplot(snr,snc,2)
        plot(obs.rft1.time_steps, obs.rft1.ttorque(:,3), 'Color', '#0072BD', 'LineWidth', 1, 'LineStyle','-'); hold on;
        plot(obs.rft1.time_steps, obs.rft2.ttorque(:,3), 'Color', '#D95319', 'LineWidth', 1, 'LineStyle','-');
        grid on; title('\tau_1^{tot} \tau_2^{tot}')
        xline(obs.tdec_sec,'-.b', 'td',...
                        'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal');
        legend({'\tau_1','\tau_2'}, 'NumColumns', 2, 'Location', 'best')
        ylabel('Nm'); 
        ytemp = ylim; ylims_torque = [ytemp];
        hold off;
        
        subplot(snr,snc,4)
        plot(obs.rft1.time_steps, obs.rft1.ttorque(:,3) + obs.rft2.ttorque(:,3), 'Color', '#0072BD', 'LineWidth', 1, 'LineStyle','-'); hold on;
        plot(obs.rft1.time_steps, obs.rft1.ttorque(:,3) - obs.rft2.ttorque(:,3), 'Color', '#D95319', 'LineWidth', 1, 'LineStyle','-'); 
        grid on; title('\tau_{sum}  \tau_{str}')
        xline(obs.tdec_sec,'-.b', 'td',...
                        'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')
        legend({'\tau_{sum}', '\tau_{str}'}, 'NumColumns', 2, 'Location', 'best')
        hold off;
        ytemp = ylim; ylims_torque = [ ylims_torque ytemp];
        
        
        
        
        subplot(snr,snc,6)
        plot(obs.imu.time_steps, obs.imu.angvel(:,3), 'LineWidth', 1)
        grid on;title('angular velocity')
        xline(obs.tdec_sec,'-.b', 'td',...
                        'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')
        ylabel('rad/sec')
        ylim([-1.1, 1.1])

        
        subplot(snr,snc,8)
        plot(obs.pose123.time_steps, obs.pose123.linvelB(:,[1,2]), 'LineWidth', 1)
        grid on; title('Linear Velocity BF');
        xline(obs.tdec_sec,'-.b', 'td',...
                        'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom','LabelOrientation','horizontal')
        xlabel('time, [sec]')
        legend({'v_x', 'v_y'}, 'NumColumns', 2, 'Location', 'best')
        ylabel('m/s')
        
        
        
        sgtitle(sprintf('%s %s %s RFT BF SE2', obs.obs_id, obs.traj_type, obs.motion_type ), ...
                    'Interpreter','none');
        
        % Set common ylims for Force plots
        subplot(snr,snc,1); ylim([min(ylims_force) max(ylims_force)])
        subplot(snr,snc,3); ylim([min(ylims_force) max(ylims_force)])
        subplot(snr,snc,5); ylim([min(ylims_force) max(ylims_force)])
        subplot(snr,snc,7); ylim([min(ylims_force) max(ylims_force)])
        % Set common ylims for Torque plots
        subplot(snr,snc,2); ylim([min(ylims_torque) max(ylims_torque)])
        subplot(snr,snc,4); ylim([min(ylims_torque) max(ylims_torque)])



end
    
end