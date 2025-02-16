%% WPI High Power Model Rocket MQP
% Author: Daniel Pearson
% Version: 2/15/2025

clear variables; close all; clc;

rawLSM303 = readtable('./data/LSM303/LSM303_2136_15_2_2025.csv');

% Extract Timestamp
time = rawLSM303.timestamp / 1000;

% Read Acceleration Values
accelX = rawLSM303.accelX; % [mg]
accelY = rawLSM303.accelY; % [mg]
accelZ = rawLSM303.accelZ; % [mg]

%% Plot Readings

figure('Name', 'Acceleration Readings');
plot(time, accelX, 'DisplayName', 'Accel X');
hold on;
plot(time, accelY, 'DisplayName', 'Accel Y');
plot(time, accelZ, 'DisplayName', 'Accel Z');
hold off;
grid on;
title('Raw Acceleration Readings');
xlabel('Time (s)');
ylabel('Acceleration (mg)');
legend()

%% Calculate Sensor Error
% Root Mean Squared Error (RMS)
RMS_X = std(accelX); % [mg]
RMS_Y = std(accelY); % [mg]
RMS_Z = std(accelZ); % [mg]

% Velocity Random Walk (VRW)
dt = mean(diff(time));
fs = 1 / dt;

VRW_x = RMS_X / sqrt(fs);
VRW_y = RMS_Y / sqrt(fs);
VRW_z = RMS_Z / sqrt(fs);

%% Gaussian Distribution
% Calculate deviation
dev_accelX = accelX - mean(accelX);
dev_accelY = accelY - mean(accelY);
dev_accelZ = accelZ - mean(accelZ);

% Fit probability density function
pd_accelX = fitdist(dev_accelX, 'Normal');
pd_accelY = fitdist(dev_accelY, 'Normal');
pd_accelZ = fitdist(dev_accelZ, 'Normal');

% 100 Point resolution for distribution x bounds
distRange_accel = linspace(min([dev_accelX; dev_accelY; dev_accelZ]), max([dev_accelX; dev_accelY; dev_accelZ]), 100);

pd_accelX = pdf(pd_accelX, distRange_accel);
pd_accelY = pdf(pd_accelY, distRange_accel);
pd_accelZ = pdf(pd_accelZ, distRange_accel);

% Plot Gaussian Distribution
figure('Name', 'LSM303 Error Distribution');

% Acceleration X Error
subplot(3,1,1);
hold on;
histogram(dev_accelX, 'Normalization', 'pdf', 'FaceAlpha', 0.5, 'DisplayName', 'Accel X Error');
plot(distRange_accel, pd_accelX, 'r', 'LineWidth', 2, 'DisplayName', 'X Fit');
hold off;
grid on;
xlabel('Acceleration Error (mg)');
ylabel('Probability Density');
title('Acceleration X Error Distribution');
legend();

% Acceleration Y Error
subplot(3,1,2);
hold on;
histogram(dev_accelY, 'Normalization', 'pdf', 'FaceAlpha', 0.5, 'DisplayName', 'Accel Y Error');
plot(distRange_accel, pd_accelY, 'g', 'LineWidth', 2, 'DisplayName', 'Y Fit');
hold off;
grid on;
xlabel('Acceleration Error (mg)');
ylabel('Probability Density');
title('Acceleration Y Error Distribution');
legend();

% Acceleration Z Error
subplot(3,1,3);
hold on;
histogram(dev_accelZ, 'Normalization', 'pdf', 'FaceAlpha', 0.5, 'DisplayName', 'Accel Z Error');
plot(distRange_accel, pd_accelZ, 'b', 'LineWidth', 2, 'DisplayName', 'Z Fit');
hold off;
grid on;
xlabel('Acceleration Error (mg)');
ylabel('Probability Density');
title('Acceleration Z Error Distribution');
legend();
