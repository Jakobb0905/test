clear all
clear
close all

outputDirectory = '..\img';

[timeVector1, voltageVector1] = importAgilentBin('./DriveLabSession4/scope_41.bin', 2);

% voltageVector1 = voltageVector1 - 0.222;

figure('Name','Phase A Current 1');
t_ms = timeVector1 * 1000;
plot(t_ms, voltageVector1);
grid on;
xticks(-50:10:50);
xlabel('Time (ms)');
ylabel('Current (A)');
saveas(gcf, fullfile(outputDirectory, 'foc_current.png'));

fs = 1 / (timeVector1(2) - timeVector1(1)); % Sampling frequency
y = lowpass(voltageVector1,2000,fs,ImpulseResponse="iir",Steepness=0.95);

plot(t_ms, voltageVector1, t_ms, y);
grid on;
legend('show');
legend('Original','Filtered');
xticks(-50:10:50);
xlabel('Time (ms)');
ylabel('Current (A)');
saveas(gcf, fullfile(outputDirectory, 'foc_current_lp.png'));




% Extract the time and data from the timeseries object
t = timeVector1;
x = voltageVector1;


% Compute the sampling frequency assuming equidistant time points
Fs = 1 / mean(diff(t));

% Compute the FFT
Y = fft(x);

% Set DC component to 0 as we meassured to Uref
Y(1)=0;

% Compute the two-sided spectrum P2 and the single-sided spectrum P1
P2 = abs(Y / length(x));
P1 = P2(1:length(x)/2 + 1);
P1(2:end-1) = 2 * P1(2:end-1);

% Define the frequency domain f
f = Fs * (0:(length(x)/2)) / length(x);

% Plot single-sided amplitude spectrum
figure('Name','Single-Sided Amplitude Spectrum of A Current');
p = loglog(f, P1);
grid on;
xlabel('Frequency (Hz)');
ylabel('|Y(f)|');
saveas(gcf, fullfile(outputDirectory, 'foc_current_fft.png'));






x = x';
% Zero-padding: extend x with zeros to make its length 4 times the original
N_original = length(x);
N_padded = 4 * N_original;
x_padded = [x zeros(1, N_padded - N_original)];

% Compute the FFT of the zero-padded signal
Y_padded = fft(x_padded);
% Y_padded(1:3) = [0,0,0]

% Compute the two-sided spectrum P2 and the single-sided spectrum P1
P2_padded = abs(Y_padded / N_padded);
P1_padded = P2_padded(1:N_padded / 2 + 1);
P1_padded(2:end-1) = 2 * P1_padded(2:end-1);

% Define the frequency domain f for the zero-padded signal
f_padded = Fs * (0:(N_padded / 2)) / N_padded;

idx = 14:4:(14+10*4);
idx = [15, 21, 26, 29, 32]
f_THD_padded = f_padded(idx);
P_THD_padded = P1_padded(idx);
% calculate THD in percent
THD_F_current = rssq(P_THD_padded(2:end))/P_THD_padded(1)*100

% (Optional) Plot the single-sided amplitude spectrum
figure;
p = loglog(f_padded, P1_padded, f_THD_padded, P_THD_padded);
grid on;
p(2).Marker='o';
p(2).LineStyle='none';
xlabel('Frequency (f)');
ylabel('|P1(f)|');

saveas(gcf, fullfile(outputDirectory, 'foc_current_fft_padded.png'));













[timeVector1, voltageVector1] = importAgilentBin('./DriveLabSession4/scope_41.bin', 1);

bcvoltage = voltageVector1;
figure('Name','Phase B-C');
t_ms = timeVector1 * 1000;
plot(t_ms, bcvoltage);
grid on;
xticks(-50:10:50);
xlabel('Time (ms)');
ylabel('Voltage (V)');
saveas(gcf, fullfile(outputDirectory, 'foc_voltage.png'));


fs = 1 / (timeVector1(2) - timeVector1(1)); % Sampling frequency
y = lowpass(voltageVector1,2000,fs,ImpulseResponse="iir",Steepness=0.95);

plot(t_ms, voltageVector1, t_ms, y);
grid on;
legend('show');
legend('Original','Filtered');
xticks(-50:10:50);
xlabel('Time (ms)');
ylabel('Voltage (V)');
saveas(gcf, fullfile(outputDirectory, 'foc_voltage_lp.png'));




% Extract the time and data from the timeseries object
t = timeVector1;
x = bcvoltage ;

% Compute the sampling frequency assuming equidistant time points
Fs = 1 / mean(diff(t));

% Compute the FFT
Y = fft(x);

% Compute the two-sided spectrum P2 and the single-sided spectrum P1
P2 = abs(Y / length(x));
P1 = P2(1:length(x)/2 + 1);
P1(2:end-1) = 2 * P1(2:end-1);

% Define the frequency domain f
f = Fs * (0:(length(x)/2)) / length(x);





x = x';
% Zero-padding: extend x with zeros to make its length 4 times the original
N_original = length(x);
N_padded = 4 * N_original;
x_padded = [x zeros(1, N_padded - N_original)];

% Compute the FFT of the zero-padded signal
Y_padded = fft(x_padded);

% Compute the two-sided spectrum P2 and the single-sided spectrum P1
P2_padded = abs(Y_padded / N_padded);
P1_padded = P2_padded(1:N_padded / 2 + 1);
P1_padded(2:end-1) = 2 * P1_padded(2:end-1);

% Define the frequency domain f for the zero-padded signal
f_padded = Fs * (0:(N_padded / 2)) / N_padded;

idx = 14:4:(14+10*4);
idx = [15, 20, 24, 27, 31]
f_THD_padded = f_padded(idx);
P_THD_padded = P1_padded(idx);
% calculate THD in percent
THD_F_voltage = rssq(P_THD_padded(2:end))/P_THD_padded(1)*100

% (Optional) Plot the single-sided amplitude spectrum
figure;
p = loglog(f_padded, P1_padded, f_THD_padded, P_THD_padded);
grid on;
p(2).Marker='o';
p(2).LineStyle='none';
xlabel('Frequency (f)');
ylabel('|P1(f)|');

saveas(gcf, fullfile(outputDirectory, 'foc_voltageBC_fft_padded.png'));


% Plot single-sided amplitude spectrum
figure;
title('Single-Sided Amplitude Spectrum of BC Phase');
p = loglog(f, P1);
grid on;
xlabel('Frequency (Hz)');
ylabel('|Y(f)|');

saveas(gcf, fullfile(outputDirectory, 'foc_voltageBC_fft.png'));




























[timeVector1, voltageVector1] = importAgilentBin('./DriveLabSession4/scope_43.bin', 2);

% voltageVector1 = voltageVector1 - 0.222;

figure('Name','Phase A Current 1');
t_ms = timeVector1 * 1000;
plot(t_ms, voltageVector1);
grid on;
xticks(-50:10:50);
xlabel('Time (ms)');
ylabel('Current (A)');
saveas(gcf, fullfile(outputDirectory, 'foc_load_current.png'));

fs = 1 / (timeVector1(2) - timeVector1(1)); % Sampling frequency
y = lowpass(voltageVector1,2000,fs,ImpulseResponse="iir",Steepness=0.95);

plot(t_ms, voltageVector1, t_ms, y);
grid on;
legend('show');
legend('Original','Filtered');
xticks(-50:10:50);
xlabel('Time (ms)');
ylabel('Current (A)');
saveas(gcf, fullfile(outputDirectory, 'foc_load_current_lp.png'));




% Extract the time and data from the timeseries object
t = timeVector1;
x = voltageVector1;


% Compute the sampling frequency assuming equidistant time points
Fs = 1 / mean(diff(t));

% Compute the FFT
Y = fft(x);

% Set DC component to 0 as we meassured to Uref
Y(1)=0;

% Compute the two-sided spectrum P2 and the single-sided spectrum P1
P2 = abs(Y / length(x));
P1 = P2(1:length(x)/2 + 1);
P1(2:end-1) = 2 * P1(2:end-1);

% Define the frequency domain f
f = Fs * (0:(length(x)/2)) / length(x);

% Plot single-sided amplitude spectrum
figure('Name','Single-Sided Amplitude Spectrum of A Current');
p = loglog(f, P1);
grid on;
xlabel('Frequency (Hz)');
ylabel('|Y(f)|');
saveas(gcf, fullfile(outputDirectory, 'foc_load_current_fft.png'));






x = x';
% Zero-padding: extend x with zeros to make its length 4 times the original
N_original = length(x);
N_padded = 4 * N_original;
x_padded = [x zeros(1, N_padded - N_original)];

% Compute the FFT of the zero-padded signal
Y_padded = fft(x_padded);
% Y_padded(1:3) = [0,0,0]

% Compute the two-sided spectrum P2 and the single-sided spectrum P1
P2_padded = abs(Y_padded / N_padded);
P1_padded = P2_padded(1:N_padded / 2 + 1);
P1_padded(2:end-1) = 2 * P1_padded(2:end-1);

% Define the frequency domain f for the zero-padded signal
f_padded = Fs * (0:(N_padded / 2)) / N_padded;

idx = 14:4:(14+10*4);
idx = [15, 21, 26, 29, 32]
f_THD_padded = f_padded(idx);
P_THD_padded = P1_padded(idx);
% calculate THD in percent
THD_F_current = rssq(P_THD_padded(2:end))/P_THD_padded(1)*100

% (Optional) Plot the single-sided amplitude spectrum
figure;
p = loglog(f_padded, P1_padded, f_THD_padded, P_THD_padded);
grid on;
p(2).Marker='o';
p(2).LineStyle='none';
xlabel('Frequency (f)');
ylabel('|P1(f)|');

saveas(gcf, fullfile(outputDirectory, 'foc_load_current_fft_padded.png'));













[timeVector1, voltageVector1] = importAgilentBin('./DriveLabSession4/scope_43.bin', 1);

bcvoltage = voltageVector1;
figure('Name','Phase B-C');
t_ms = timeVector1 * 1000;
plot(t_ms, bcvoltage);
grid on;
xticks(-50:10:50);
xlabel('Time (ms)');
ylabel('Voltage (V)');
saveas(gcf, fullfile(outputDirectory, 'foc_load_voltage.png'));


fs = 1 / (timeVector1(2) - timeVector1(1)); % Sampling frequency
y = lowpass(voltageVector1,2000,fs,ImpulseResponse="iir",Steepness=0.95);

plot(t_ms, voltageVector1, t_ms, y);
grid on;
legend('show');
legend('Original','Filtered');
xticks(-50:10:50);
xlabel('Time (ms)');
ylabel('Voltage (V)');
saveas(gcf, fullfile(outputDirectory, 'foc_load_voltage_lp.png'));




% Extract the time and data from the timeseries object
t = timeVector1;
x = bcvoltage ;

% Compute the sampling frequency assuming equidistant time points
Fs = 1 / mean(diff(t));

% Compute the FFT
Y = fft(x);

% Compute the two-sided spectrum P2 and the single-sided spectrum P1
P2 = abs(Y / length(x));
P1 = P2(1:length(x)/2 + 1);
P1(2:end-1) = 2 * P1(2:end-1);

% Define the frequency domain f
f = Fs * (0:(length(x)/2)) / length(x);





x = x';
% Zero-padding: extend x with zeros to make its length 4 times the original
N_original = length(x);
N_padded = 4 * N_original;
x_padded = [x zeros(1, N_padded - N_original)];

% Compute the FFT of the zero-padded signal
Y_padded = fft(x_padded);

% Compute the two-sided spectrum P2 and the single-sided spectrum P1
P2_padded = abs(Y_padded / N_padded);
P1_padded = P2_padded(1:N_padded / 2 + 1);
P1_padded(2:end-1) = 2 * P1_padded(2:end-1);

% Define the frequency domain f for the zero-padded signal
f_padded = Fs * (0:(N_padded / 2)) / N_padded;

idx = 14:4:(14+10*4);
idx = [15, 20, 24, 27, 31]
f_THD_padded = f_padded(idx);
P_THD_padded = P1_padded(idx);
% calculate THD in percent
THD_F_voltage = rssq(P_THD_padded(2:end))/P_THD_padded(1)*100

% (Optional) Plot the single-sided amplitude spectrum
figure;
p = loglog(f_padded, P1_padded, f_THD_padded, P_THD_padded);
grid on;
p(2).Marker='o';
p(2).LineStyle='none';
xlabel('Frequency (f)');
ylabel('|P1(f)|');

saveas(gcf, fullfile(outputDirectory, 'foc_load_voltageBC_fft_padded.png'));


% Plot single-sided amplitude spectrum
figure;
title('Single-Sided Amplitude Spectrum of BC Phase');
p = loglog(f, P1);
grid on;
xlabel('Frequency (Hz)');
ylabel('|Y(f)|');

saveas(gcf, fullfile(outputDirectory, 'foc_load_voltageBC_fft.png'));