%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Mathieu Bresciani
% Description:
%   
%
% Dependencies:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all


%% Extract measurements

mask = isnan(ret{1}.measurements(5,:));
j = 1;
t = ret{1}.time;
for i = 1:length(ret{1}.measurements(5,:))
    if mask(i) == 0
        meas(:,j) = ret{1}.measurements(:,i);
        t_meas(j) = ret{1}.time(i);
        j = j+1;
    end
end

mask_gps = isnan(ret{1}.measurements(1,:));
t_gps = t(~mask_gps);
figure; plot(t,ret{1}.inputTrajectory(1,:),t_gps,ret{1}.measurements(1,~mask_gps))
delay = ret{1}.inputTrajectory(1,~mask_gps) - ret{1}.measurements(1,~mask_gps);
figure; stem(t_gps, delay);
xlabel('Time (s)');
ylabel('Delay (s)');

%%
figure;
plot(t,t)
hold all
plot(t,ret{1}.realTime)