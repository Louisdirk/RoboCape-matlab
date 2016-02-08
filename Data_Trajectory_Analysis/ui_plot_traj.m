function ui_plot_traj(t_meas, meas, t_est, gpsEst, vfEst, yawEst, traj_ref)

Ts = t_est(2) - t_est(1);

% Plot GPS measurements
c = linspace(1,10,length(meas));
h1 = scatter(meas(1,:), meas(2,:), [], c);
hold all
h2 = quiver(meas(1,:), meas(2,:), 0.1*cos(meas(6,:)), 0.1*sin(meas(6,:)));

% Plot position estimate
h3 = plot(gpsEst(1,:), gpsEst(2,:), '.');
quiverDisplay = 1:10:length(gpsEst);
h4 = quiver(gpsEst(1,quiverDisplay), gpsEst(2,quiverDisplay), vfEst(quiverDisplay).*cos(yawEst(quiverDisplay)), vfEst(quiverDisplay).*sin(yawEst(quiverDisplay)));
title('Position');
xlabel('x^l');
ylabel('y^l');

% Plot reference path
h5 = plot(traj_ref(1,:),traj_ref(2,:));

% Plot car position logo every second
k = 0;
timeLogo = 0:floor(1/Ts)*Ts:t_est(end-2);
indexLogo = 1:floor(1/Ts):length(t_est)-2;
for i = indexLogo
    k = k+1;
    origin = gpsEst(:,i);
    phi = yawEst(i);
    scale = 0.2;
    logo{k} = getPositionLogo(origin, phi, scale);
    hLogo(k) = plot(logo{k}(1,:),logo{k}(2,:),'k');
    
    dPos = traj_ref(:,indexLogo(k)+2)-traj_ref(:,indexLogo(k));
    phiRef = atan2(dPos(2),dPos(1));
    logoRef{k} = getPositionLogo(traj_ref(:,indexLogo(k)), phiRef, scale);
    hLogoRef(k) = plot(logoRef{k}(1,:),logoRef{k}(2,:),'Color',[0.8 0.8 0.8]);
end

axis equal
axis manual


span = [0 t_est(end)];
varname = 'Time';
hs = uicontrol(...
    'style','slider',...
    'units', 'normalized', ...
    'pos',[0.1 0 0.8 0.05],...
    'min',span(1),...
    'max',span(2),...
    'value',span(2),...
    'SliderStep',[Ts/t_est(end) Ts],...
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
        set(h5, 'XData', traj_ref(1,1:iMax_t), 'YData', traj_ref(2,1:iMax_t));
        
        for k = 1:length(timeLogo)
           if  timeLogo(k) <= maxT
               set(hLogo(k),'Visible','on')
               set(hLogoRef(k),'Visible','on')
           else
               set(hLogo(k),'Visible','off')
               set(hLogoRef(k),'Visible','off')
           end
        end

        drawnow;
    end

end