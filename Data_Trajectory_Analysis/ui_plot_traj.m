function ui_plot_traj(t_meas, meas, t_est, gpsEst, vfEst, yawEst)

% Plot GPS measurements
c = linspace(1,10,length(meas));
h1 = scatter(meas(1,:), meas(2,:), [], c);
hold all
h2 = quiver(meas(1,:), meas(2,:), 0.1*cos(meas(6,:)), 0.1*sin(meas(6,:)));

% Plot GPS estimate
h3 = plot(gpsEst(1,:), gpsEst(2,:), '.');
quiverDisplay = 1:10:length(gpsEst);
h4 = quiver(gpsEst(1,quiverDisplay), gpsEst(2,quiverDisplay), vfEst(quiverDisplay).*cos(yawEst(quiverDisplay)), vfEst(quiverDisplay).*sin(yawEst(quiverDisplay)));
title('Position');
xlabel('x^l');
ylabel('y^l');

span = [0 t_est(end)];
varname = 'Time';
hs = uicontrol(...
    'style','slider',...
    'units', 'normalized', ...
    'pos',[0.1 0 0.8 0.05],...
    'min',span(1),...
    'max',span(2),...
    'value',span(2),...
    'callback',@sliderupdate,...
    'tag',varname);

ht = uicontrol('style','text');
set(ht, 'String', ['t = ' num2str(t_est(end))]);

    function sliderupdate(a,b)
        maxT = a.Value;
        set(ht, 'String', ['t = ' num2str(maxT)]);
        [cc, iMax_t] = min(abs(t_est-maxT));
        [cc, iMax_t_meas] = min(abs(t_meas-maxT));
        set(h1, 'XData', meas(1,1:iMax_t_meas), 'YData', meas(2,1:iMax_t_meas), 'CData', c(1:iMax_t_meas));
        set(h2, 'XData', meas(1,1:iMax_t_meas), 'YData', meas(2,1:iMax_t_meas), 'UData', 0.1*cos(meas(6,1:iMax_t_meas)), 'VData', 0.1*sin(meas(6,1:iMax_t_meas)));
        set(h3, 'XData', gpsEst(1,1:iMax_t), 'YData', gpsEst(2,1:iMax_t));
        quiverDisplay = 1:10:iMax_t;
        set(h4, 'XData', gpsEst(1,quiverDisplay), 'YData', gpsEst(2,quiverDisplay), 'UData', vfEst(quiverDisplay).*cos(yawEst(quiverDisplay)), 'VData', vfEst(quiverDisplay).*sin(yawEst(quiverDisplay)));
        drawnow;
    end

end