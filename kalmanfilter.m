%Kalman filter
% Simulation parameters
numSamples = 100;  % Number of time steps
% System parameters
trueChannelParams = [1.2; 0.8];  % True channel parameters (example: gain and delay)
processNoise = 0.01;             % Process noise covariance
measurementNoise = 0.1;          % Measurement noise covariance
% State-space representation
A = eye(2);                      % State transition matrix (identity matrix)
H = eye(2);                      % Measurement matrix (identity matrix)
Q = processNoise * eye(2);       % Process noise covariance matrix
R = measurementNoise * eye(2);   % Measurement noise covariance matrix
% Kalman filter initialization
initialStateEstimate = [1; 1];   % Initial guess for channel parameters
initialStateCovariance = eye(2); % Initial guess for covariance matrix
% Initialize arrays to store true and estimated channel parameters
trueParamsHistory = zeros(2, numSamples);
estimatedParamsHistory = zeros(2, numSamples);
% Kalman filter initialization
x_hat = initialStateEstimate;
P = initialStateCovariance;
% Simulate data and apply Kalman filter
for k = 1:numSamples
    % Simulate true channel parameters (e.g., random walk)
    trueChannelParams = trueChannelParams + sqrt(Q) * randn(2, 1);
    
    % Generate noisy measurements
    measurement = trueChannelParams + sqrt(R) * randn(2, 1);
    
    % Kalman filter prediction
    x_hat_minus = A * x_hat;
    P_minus = A * P * A' + Q;
    
    % Kalman filter update
    K = P_minus * H' / (H * P_minus * H' + R);
    x_hat = x_hat_minus + K * (measurement - H * x_hat_minus);
    P = (eye(2) - K * H) * P_minus;
    
    % Store true and estimated parameters
    trueParamsHistory(:, k) = trueChannelParams;
    estimatedParamsHistory(:, k) = x_hat;
end
% Calculate tracking error
trackingError = sqrt(sum((estimatedParamsHistory - trueParamsHistory).^2, 1));
% Plot results
figure;
subplot(2,1,1);
plot(trueParamsHistory(1, :), 'b', 'LineWidth', 2, 'DisplayName', 'True Gain');
hold on;
plot(estimatedParamsHistory(1, :), 'r--', 'LineWidth', 2, 'DisplayName', 'Estimated Gain');
xlabel('Time Step');
ylabel('Gain');
legend('show');
title('Channel Tracking - Gain');
subplot(2,1,2);
plot(trueParamsHistory(2, :), 'b', 'LineWidth', 2, 'DisplayName', 'True Delay');
hold on;
plot(estimatedParamsHistory(2, :), 'r--', 'LineWidth', 2, 'DisplayName', 'Estimated Delay');
xlabel('Time Step');
ylabel('Delay');
legend('show');
title('Channel Tracking - Delay');
figure;
plot(trackingError, 'k', 'LineWidth', 2);
xlabel('Time Step');
ylabel('Tracking Error');
title('Kalman Filter Tracking Error');