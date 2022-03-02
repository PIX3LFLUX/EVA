clc;
clear;
close all;

%% Calibration of accelerometer based on the method described in
% https://www.st.com/resource/en/application_note/an4508-parameters-and-calibration-of-a-lowg-3axis-accelerometer-stmicroelectronics.pdf
% Datasets are colleced, but instead of solving a bigger matrix operation,
% the data is averaged. This has the advantage of being much easier to
% understand. The results should be the same. To make calibration easier,
% it remains a manual process, which is not a problem since a calibrated
% car stays calibrated and does not need to be re-calibrated unless damaged
% or otherwise physically changed.

% Program settings for sensorcar (globals.h):
% DEBUG                     0 for fast data sampling
% MEASURE_SYSTEM            0 to log normally and for as long as desired
% CALIBRATE_ACCELERATION    0 to read the raw values
% DATA_LOGGING              1 to log data to the sd card

% Wiping the sd card before starting is recommended.
% Import the data as column vectors using the matlab import tool, then save
% the workspace as mat file.

%% load data
load cal_raw_data.mat

%% Plotting raw data
%{
Time = Time / 1e6;
figure();
hold on;
grid on;
title('Raw values front')
plot(Time,Accel_Front_x)
plot(Time,Accel_Front_y)
plot(Time,Accel_Front_z)
xlabel('Time in s')
ylabel('Amplitude in LSBs')
legend('Accel Front x', 'Accel Front y', 'Accel Front z')
%}

%% Getting data normalized to local gravity
% this is not required, since the calibration will deal with this if the
% true values are normalized already.
% range is +-2g @ 16Bit signed (+32767 counts, -32768 counts)
%{
range_measured = 8;  %+- max g of scale that was used measuring the calibration
Accel_Front_x = Accel_Front_x / 32768 * range_measured;
Accel_Front_y = Accel_Front_y / 32768 * range_measured;
Accel_Front_z = Accel_Front_z / 32768 * range_measured;

Accel_Heck_x = Accel_Heck_x / 32768 * range_measured;
Accel_Heck_y = Accel_Heck_y / 32768 * range_measured;
Accel_Heck_z = Accel_Heck_z / 32768 * range_measured;
%}

%% Averaging data to see what indices need to be used to get the right measurements in time
FILTER_LENGTH = 1e2;
lowpass_filter_kernel = 1/FILTER_LENGTH * ones(FILTER_LENGTH,1);

Accel_Front_x = conv(Accel_Front_x, lowpass_filter_kernel);
Accel_Front_y = conv(Accel_Front_y, lowpass_filter_kernel);
Accel_Front_z = conv(Accel_Front_z, lowpass_filter_kernel);

Accel_Heck_x  = conv(Accel_Heck_x, lowpass_filter_kernel);
Accel_Heck_y  = conv(Accel_Heck_y, lowpass_filter_kernel);
Accel_Heck_z  = conv(Accel_Heck_z, lowpass_filter_kernel);

figure();
subplot(2,2,1)
hold on;
grid on;
title('Smoothed values front')
plot(Accel_Front_x)
plot(Accel_Front_y)
plot(Accel_Front_z)
xlabel('Index of the sample')
ylabel('Amplitude in counts (16Bit signed)')
legend('Accel x', 'Accel y', 'Accel z')

subplot(2,2,2)
hold on;
grid on;
title('Smoothed values back')
plot(Accel_Heck_x)
plot(Accel_Heck_y)
plot(Accel_Heck_z)
xlabel('Index of the sample')
ylabel('Amplitude in counts (16Bit signed)')
legend('Accel x', 'Accel y', 'Accel z')

ans = "The breakpoint is here to set the indices of the six measurement points."
dbstop in accelerometer_calibration_calculations.m at 94
clear ans;

% indices (this part is manual labor). Look at the smoothed values and pick
% an index at a measuring point that seems representative. These are the
% axes of the CAR. So take note when you orient the calibration rig. Which
% axes are you calibrating in which order? For ease of use, do +x, -x, +-y,
% +-z. +x means the car nose is pointing towards the SKY, not the earth. +y
% is the left side of the car (in driving direction) facing earth.
%+-8g
index_plus_x  = 988;
index_minus_x = 6989;
index_plus_y  = 14888;
index_minus_y = 22893;
index_plus_z  = 31899;
index_minus_z = 38905;

%% Measured Data
% rows are datapoints, columns are measured x, measured y, measured z and
% 1 (the number). Equivalent of w in the application note.
accel_front_means = ...
[...
    Accel_Front_x(index_plus_x)   Accel_Front_y(index_plus_x)   Accel_Front_z(index_plus_x)     1; ...%+x vehicle
    Accel_Front_x(index_minus_x)  Accel_Front_y(index_minus_x)  Accel_Front_z(index_minus_x)    1; ...%-x
    Accel_Front_x(index_plus_y)   Accel_Front_y(index_plus_y)   Accel_Front_z(index_plus_y)     1; ...%+y 
    Accel_Front_x(index_minus_y)  Accel_Front_y(index_minus_y)  Accel_Front_z(index_minus_y)    1; ...%-y
    Accel_Front_x(index_plus_z)   Accel_Front_y(index_plus_z)   Accel_Front_z(index_plus_z)     1; ...%+z
    Accel_Front_x(index_minus_z)  Accel_Front_y(index_minus_z)  Accel_Front_z(index_minus_z)    1; ...%-z
];

accel_back_means = ...
[...
    Accel_Heck_x(index_plus_x)   Accel_Heck_y(index_plus_x)   Accel_Heck_z(index_plus_x)     1; ...%+x vehicle
    Accel_Heck_x(index_minus_x)  Accel_Heck_y(index_minus_x)  Accel_Heck_z(index_minus_x)    1; ...%-x
    Accel_Heck_x(index_plus_y)   Accel_Heck_y(index_plus_y)   Accel_Heck_z(index_plus_y)     1; ...%+y
    Accel_Heck_x(index_minus_y)  Accel_Heck_y(index_minus_y)  Accel_Heck_z(index_minus_y)    1; ...%-y    
    Accel_Heck_x(index_plus_z)   Accel_Heck_y(index_plus_z)   Accel_Heck_z(index_plus_z)     1; ...%+z
    Accel_Heck_x(index_minus_z)  Accel_Heck_y(index_minus_z)  Accel_Heck_z(index_minus_z)    1; ...%-z
];

% those are the accelerations that were applied during calibration. Order
% is x-y-z. Equivalent of Y in the application note.
reference_accels = ...
[...
    1 0 0;
    -1 0 0;
    0 1 0;
    0 -1 0;
    0 0 1;
    0 0 -1;
];

%% Math
% least square method calculation X = [w.T * w]^(-1) * w.T * Y
% corrected values are
% x_cal = x_measured * factors(1,1) + y_measured * factors(2,1) + z_measured * factors(3,1) + factors(4,1)
% y_cal = x_measured * factors(1,2) + y_measured * factors(2,2) + z_measured * factors(3,2) + factors(4,2)
% z_cal = x_measured * factors(1,3) + y_measured * factors(2,3) + z_measured * factors(3,3) + factors(4,3)
correction_factors_front = (transpose(accel_front_means)*accel_front_means)\transpose(accel_front_means)*reference_accels;
correction_factors_back  = (transpose(accel_back_means)*accel_back_means)\transpose(accel_back_means)*reference_accels;

accel_front_x_cal = [Accel_Front_x Accel_Front_y Accel_Front_z ones(length(Accel_Front_x),1)] * (correction_factors_front(:,1));
accel_front_y_cal = [Accel_Front_x Accel_Front_y Accel_Front_z ones(length(Accel_Front_x),1)] * (correction_factors_front(:,2));
accel_front_z_cal = [Accel_Front_x Accel_Front_y Accel_Front_z ones(length(Accel_Front_x),1)] * (correction_factors_front(:,3));

accel_back_x_cal = [Accel_Heck_x Accel_Heck_y Accel_Heck_z ones(length(Accel_Front_x),1)] * (correction_factors_back(:,1));
accel_back_y_cal = [Accel_Heck_x Accel_Heck_y Accel_Heck_z ones(length(Accel_Front_x),1)] * (correction_factors_back(:,2));
accel_back_z_cal = [Accel_Heck_x Accel_Heck_y Accel_Heck_z ones(length(Accel_Front_x),1)] * (correction_factors_back(:,3));

accel_front_cal_matrix = ...
[...
    accel_front_x_cal(index_plus_x)   accel_front_y_cal(index_plus_x)   accel_front_z_cal(index_plus_x)  ; ...%+x
    accel_front_x_cal(index_minus_x)  accel_front_y_cal(index_minus_x)  accel_front_z_cal(index_minus_x) ; ...%-x
    accel_front_x_cal(index_plus_y)   accel_front_y_cal(index_plus_y)   accel_front_z_cal(index_plus_y)  ; ...%+y vehicle
    accel_front_x_cal(index_minus_y)  accel_front_y_cal(index_minus_y)  accel_front_z_cal(index_minus_y) ; ...%-y    
    accel_front_x_cal(index_plus_z)   accel_front_y_cal(index_plus_z)   accel_front_z_cal(index_plus_z)  ; ...%+z
    accel_front_x_cal(index_minus_z)  accel_front_y_cal(index_minus_z)  accel_front_z_cal(index_minus_z) ; ...%-z
];

accel_back_cal_matrix = ...
[...
    accel_back_x_cal(index_plus_x)   accel_back_y_cal(index_plus_x)   accel_back_z_cal(index_plus_x)  ; ...%+x
    accel_back_x_cal(index_minus_x)  accel_back_y_cal(index_minus_x)  accel_back_z_cal(index_minus_x) ; ...%-x
    accel_back_x_cal(index_plus_y)   accel_back_y_cal(index_plus_y)   accel_back_z_cal(index_plus_y)  ; ...%+y vehicle
    accel_back_x_cal(index_minus_y)  accel_back_y_cal(index_minus_y)  accel_back_z_cal(index_minus_y) ; ...%-y    
    accel_back_x_cal(index_plus_z)   accel_back_y_cal(index_plus_z)   accel_back_z_cal(index_plus_z)  ; ...%+z
    accel_back_x_cal(index_minus_z)  accel_back_y_cal(index_minus_z)  accel_back_z_cal(index_minus_z) ; ...%-z
];

min_square_error_front  = 1/9 * sum(sum( (reference_accels - accel_front_cal_matrix).^2 ))
min_square_error_back   = 1/9 * sum(sum( (reference_accels - accel_back_cal_matrix).^2 ))

subplot(2,2,3)
hold on;
grid on;
title('Smoothed and corrected values front')
plot(accel_front_x_cal)
plot(accel_front_y_cal)
plot(accel_front_z_cal)
xlabel('Index of the sample')
ylabel('Amplitude in factors of g (9.81m/s^2)')
legend('Accel Front x', 'Accel Front y', 'Accel Front z')

subplot(2,2,4)
hold on;
grid on;
title('Smoothed and corrected values back')
plot(accel_back_x_cal)
plot(accel_back_y_cal)
plot(accel_back_z_cal)
xlabel('Index of the sample')
ylabel('Amplitude in factors of g (9.81m/s^2)')
legend('Accel Front x', 'Accel Front y', 'Accel Front z')

%% data prep for export
range_running = 8   %+- max g of scale that is used on the car
range_measured = 8  %+- max g of scale that was used measuring the calibration
range_factor = range_running / range_measured;
correction_factors_front_exportable = reshape(correction_factors_front,[1,12]) * range_factor;
correction_factors_back_exportable = reshape(correction_factors_back,[1,12]) * range_factor;
ans = "If you choose a range different from range_measured, set range_running appropriately."
ans = "Copy the correction_factors_*_exportable from the matlab variable viewer for best accuracy."
clear ans;