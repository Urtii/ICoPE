% Example data
timestamps = groundTruthTrajectory.Time; % Non-uniform times
signal = data_log.global(1,:); % Signal values

% Define frequency range for analysis
f = linspace(0, 5, 500); % Frequency range (Hz)

% Compute Lomb-Scargle Periodogram
[pxx, f] = plomb(signal, timestamps, f);

% Plot results
figure;
plot(f, pxx);
xlabel('Frequency (Hz)');
ylabel('Power');
title('Lomb-Scargle Periodogram');
