function plot_result_part2(t,est_x, gt_x, gt_y, sigma_array)
% plot results for part 1, compare full states/measurements to linearized
% states/measurements
    t = t';

    % plot state trajectory
    figure
    title_of_plot_x = ["UGV \xi", "UGV \eta", "UGV \theta", "UAV \xi", "UAV \eta", "UAV \theta"];
    for i = 1:6
        subplot(2,3,i)
        hold on
        plot(t,gt_x(i,:), 'k') % ground-truth state trajectory
        plot(t,est_x(i,:), 'r') % estimated state trajectory from EKF
        %plot(t,est_x(i,:)+2*sigma_array(i,:),'k--')
        %plot(t,est_x(i,:)-2*sigma_array(i,:),'k--')
        legend('ground truth', 'EKF')
        title(title_of_plot_x(i))
    end 

    figure
    esimation_error = wrapx(gt_x-est_x);
    two_sigma = wrapx(2*sigma_array);
    for i = 1:6
        subplot(2,3,i)
        hold on        
        plot(t,esimation_error(i,:), 'k')
        plot(t,two_sigma(i,:),'r--')
        plot(t,-two_sigma(i,:),'r--')
        legend('esimation error', '2\sigma bound')
        title(title_of_plot_x(i))
    end 

    % plot measurement
    figure
    title_of_plot_y = ["y_1", "y_2", "y_3", "y_4", "y_5"];
    for i = 1:5
        subplot(5,1,i)
        hold on
        plot(t,gt_y(i,:), 'k') % full-dynamic measurement trajectory
        legend('noisy measurements')
        title(title_of_plot_y(i))
    end    
end
