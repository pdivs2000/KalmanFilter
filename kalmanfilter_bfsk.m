%Kalman filter bfsk
% Simulation parameters
numSymbols = 100;   % Number of BFSK symbols
snr = 15;           % Signal-to-noise ratio in dB
% Modulation parameters
f0 = 1000;          % Frequency for binary '0'
f1 = 2000;          % Frequency for binary '1'
% System parameters
trueChannelParams = [1.2; 0.8];  % True channel parameters (gain and delay)
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
trueParamsHistory = zeros(2, numSymbols);
estimatedParamsHistory = zeros(2, numSymbols);
% Kalman filter initialization
x_hat = initialStateEstimate;
P = initialStateCovariance;
% Simulate BFSK modulation and apply Kalman filter
for k = 1:numSymbols
    % Generate binary data
    data = randi([0, 1]);
    
    % BFSK modulation
    if data == 0
        modulatedSymbol = sin(2*pi*f0*k);
    else
        modulatedSymbol = sin(2*pi*f1*k);
    end
    % Simulate true channel parameters (e.g., random walk)
    trueChannelParams = trueChannelParams + sqrt(Q) * randn(2, 1);
    % Transmit through channel (apply fading)
    receivedSymbol = trueChannelParams(1) * modulatedSymbol;
    % Add noise
    noise = 10^(-snr/20) * randn(size(receivedSymbol));
    receivedSymbol = receivedSymbol + noise;
    % Measurement (received symbol)
    measurement = [real(receivedSymbol); imag(receivedSymbol)];
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
xlabel('Symbol Index');
ylabel('Gain');
legend('show');
title('Channel Tracking - Gain');
subplot(2,1,2);
plot(trueParamsHistory(2, :), 'b', 'LineWidth', 2, 'DisplayName', 'True Delay');
hold on;
plot(estimatedParamsHistory(2, :), 'r--', 'LineWidth', 2, 'DisplayName', 'Estimated Delay');
xlabel('Symbol Index');
ylabel('Delay');
legend('show');
title('Channel Tracking - Delay');
figure;
plot(trackingError, 'k', 'LineWidth', 2);
xlabel('Symbol Index');
ylabel('Tracking Error');
title('Kalman Filter Tracking Error');
has context menu