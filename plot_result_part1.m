function plot_result_part1(t,x_full, y_full, x_approx, y_approx, x_nom)
% plot results for part 1, compare full states/measurements to linearized
% states/measurements   
    t = t';
    % plot state trajectory
    figure
    title_of_plot_x = ["UGV \xi", "UGV \eta", "UGV \theta", "UAV \xi", "UAV \eta", "UAV \theta"];
    for i = 1:6
        subplot(6,1,i)
        hold on
        plot(t,x_full(i,:), 'k') % full-dynamic state trajectory
        plot(t,x_approx(i,:), 'r') % linearized DT state trajectory
        %plot(t,x_nom(i,:), 'r') 
        %legend('full dynamics', 'nominal')
        legend('full dynamics', 'linearized DT')
        title(title_of_plot_x(i))
    end 
    % plot measurement
    figure
    title_of_plot_y = ["y_1", "y_2", "y_3", "y_4", "y_5"];
    for i = 1:5
        subplot(5,1,i)
        hold on
        plot(t,y_full(i,:), 'k') % full-dynamic measurement trajectory
        plot(t,y_approx(i,:), 'r') % linearized DT measurement trajectory
        legend('full dynamics', 'linearized DT')
        title(title_of_plot_y(i))
    end    
end
