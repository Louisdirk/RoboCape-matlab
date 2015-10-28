function plot_topic(x, y, y_legend, x_limit_len)

    x_min = min(x); x_max = max(x);
    x_index_min = 1; x_index_max = length(x);

    if length(x) >= x_limit_len
        x_index_min = x_index_max - x_limit_len + 1;
        x_avg_pitch = (max(x) - min(x)) / (length(x) - 1);
        x_min = x_max - x_avg_pitch * x_limit_len;
    end

    colors = hsv(size(y,1));
    for i=1:size(y,1)
        plot(x(x_index_min:end), y(i,x_index_min:end), 'color', colors(i,:));
        legendinfo{i} = [y_legend ' ' num2str(i)];
    end

    xlim([x_min inf]);
    legend(legendinfo);
end
