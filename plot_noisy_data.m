function plot_noisy_data(t, gt_y)
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