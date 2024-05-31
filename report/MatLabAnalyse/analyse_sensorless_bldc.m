clear all
clear
close all

outputDirectory = '..\img';

[timeVector1, voltageVector1] = importAgilentBin('./DriveLabSession4/scope_33.bin', 1);
[timeVector2, voltageVector2] = importAgilentBin('./DriveLabSession4/scope_33.bin', 2);

figure('Name','Phase A 1');
plot(voltageVector1);
figure('Name','Phase A 2');
plot(voltageVector2);




[timeVector1, voltageVector1] = importAgilentBin('./LabSession3Meassurements/scope_24.bin', 1);
[timeVector2, voltageVector2] = importAgilentBin('./LabSession3Meassurements/scope_24.bin', 2);


figure('Name','Phase A Current 1');
t_ms = timeVector1 * 1000
plot(t_ms, voltageVector1);
grid on;
xticks(-50:10:50);
xlabel('Time (ms)');
ylabel('Current (A)');
saveas(gcf, fullfile(outputDirectory, 'sensored_current.png'));

figure('Name','Phase A Current 2');
plot(voltageVector2);






% Extract the time and data from the timeseries object
t = timeVector1;
x = voltageVector1;

% Compute the sampling frequency assuming equidistant time points
Fs = 1 / mean(diff(t));

thd_db = thd(x)
percent_thd = 100*(10^(thd_db/20))
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
loglog(f, P1);
grid on;
xlabel('Frequency (Hz)');
ylabel('|Y(f)|');
saveas(gcf, fullfile(outputDirectory, 'sensored_current_fft.png'));






[timeVector1, voltageVector1] = importAgilentBin('./LabSession3Meassurements/scope_25.bin', 1);
[timeVector2, voltageVector2] = importAgilentBin('./LabSession3Meassurements/scope_25.bin', 2);
% 
% start_idx = 145576 
% end_idx = 294538
% timeVector1 = timeVector1(start_idx:end_idx);
% timeVector2 = timeVector2(start_idx:end_idx);
% voltageVector1 = voltageVector1(start_idx:end_idx);
% voltageVector2 = voltageVector2(start_idx:end_idx);


bcvoltage = voltageVector1-voltageVector2;
figure('Name','Phase B-C');
t_ms = timeVector1 * 1000
plot(t_ms, bcvoltage);
grid on;
xticks(-50:10:50);
xlabel('Time (ms)');
ylabel('Voltage (V)');
saveas(gcf, fullfile(outputDirectory, 'sensored_voltage.png'));
% Extract the time and data from the timeseries object
t = timeVector1;
x = bcvoltage ;

% Compute the sampling frequency assuming equidistant time points
Fs = 1 / mean(diff(t));
thd_db = thd(x)
percent_thd = 100*(10^(thd_db/20))
% Compute the FFT
Y = fft(x);

% Compute the two-sided spectrum P2 and the single-sided spectrum P1
P2 = abs(Y / length(x));
P1 = P2(1:length(x)/2 + 1);
P1(2:end-1) = 2 * P1(2:end-1);

% Define the frequency domain f
f = Fs * (0:(length(x)/2)) / length(x);

% Plot single-sided amplitude spectrum
figure;
title('Single-Sided Amplitude Spectrum of BC Phase');
loglog(f, P1);
grid on;
xlabel('Frequency (Hz)');
ylabel('|Y(f)|');

saveas(gcf, fullfile(outputDirectory, 'sensored_voltageBC_fft.png'));

% cleanfigure;
% matlab2tikz(fullfile(outputDirectory, 'scope_20.tex'), 'showInfo', false);