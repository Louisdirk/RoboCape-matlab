%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       MPU9150 IMU Noise estimation
%
%           Mathieu Bresciani
%                2015
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


close all;
clear all;
clc;

%%
load('static_mes2.mat')

%% Plot the signals
acc_temp = acc(:,10:end);
clear acc
acc = acc_temp;
clear acc_temp
figure(1);
subplot(3,1,1); plot(acc(1,:))
ylabel('X axis (m/s^2)');
subplot(3,1,2); plot(acc(2,:))
xlabel('Samples');
ylabel('Y axis (m/s^2)');
subplot(3,1,3); plot(acc(3,:))
xlabel('Samples');
ylabel('Z axis (m/s^2)');

%%
acc_mean(1) = mean(acc(1,:));
acc_mean(2) = mean(acc(2,:));
acc_mean(3) = mean(acc(3,:));

acc_unbiased(1,:) = acc(1,:) - acc_mean(1);
acc_unbiased(2,:) = acc(2,:) - acc_mean(2);
acc_unbiased(3,:) = acc(3,:) - acc_mean(3);

%% Autocorrelation and Power Spectral Density (PSD)
for i = 1:3
    AutoCorrAcc(i,:) = Compute_Autocorr(acc_unbiased(i,:));
    PsdAcc(i,:) = Compute_PSD(acc_unbiased(i,:));
    varAcc(i) = var(acc_unbiased(i,:));
end
%% IMU2gY
%% Plot Autocorrelation and PSD
figure(3);

subplot(3,1,1); plot(AutoCorrAcc(1,:));
title('Autocorrelation function');
ylabel('Level');
subplot(3,1,2); plot(AutoCorrAcc(2,:));
ylabel('Level');
subplot(3,1,3); plot(AutoCorrAcc(3,:));
ylabel('Level');
xlabel('Steps');


figure(4);
subplot(3,1,1); plot(PsdAcc(1,:));
subplot(3,1,2); plot(PsdAcc(2,:));
subplot(3,1,3); plot(PsdAcc(3,:));

%% Verification
nbSamples = length(acc);
nbSequences = 1;

Tc = 0.01;       % Correlation time
stdDevWn = sqrt(233.32e-6);
varGMp = 1e-6;
stdDevGMp = sqrt(2*varGMp/Tc);


WhiteNoise = stdDevWn.*randn(nbSequences, nbSamples);

WhiteNoise2 = stdDevGMp.*randn(nbSequences, nbSamples);
GMp = Generate_1st_Order_Markov(WhiteNoise2, Tc);

Simu = WhiteNoise+GMp;


AutoCorrSimu = Compute_Autocorr(Simu);
PsdSimu = Compute_PSD(Simu);

figure(3);
hold all;
for i = 1:nbSequences
    plot(AutoCorrSimu(i,:));
end
title('Autocorrelation function');
xlabel('Steps');
ylabel('Level');
hold off;
% axis([1.894e5 1.895e5 -0.5e-5 3e-5])
figure(4);
hold all;
for i = 1:nbSequences
   plot(PsdSimu(i)); 
end
hold off;
