clc;
clear;
close all;

%% overview
% this is a matlab script to determine an algorithm to determine latency 
% to the carrera control unit. If latency is determined, synchonization 
% can be attempted by waiting.

load latency_data.mat
% load latency_data2.mat
% load latency_data3.mat
% load latency_data4.mat

sampling_frequency = 100; % in Hz
sampling_interval = 1/sampling_frequency; % in s

accel_estimate = Accel_Front_x + Accel_Heck_x / 2;

figure();
hold on
plot(accel_estimate)
plot(Target_Speed/max(Target_Speed))
xlabel("Sample")
ylabel("Amplidude")
legend("Measured acceleration in factors of g","Target speed (digital, scaled down)")

find_latency(Target_Speed, accel_estimate, 0.1)

%% functions
% target_vector is speed set
% actual_vector is acceleration get
% deviation_threshold is the absolute difference two samples should have to
% be classified as a system reaction to the target_vector (a value of 10
% means that a sample needs to be 10 apart from the previos one to be 
% classified as a system response)
function [] = find_latency(target_vector, actual_vector, deviation_threshold)
    logic_vector = target_vector > 0;
    previous_logic = false;
    previous_value = actual_vector(1);
    already_calculated_flag = false;

    for ii=1:1:length(actual_vector)
        if logic_vector(ii)
            if logic_vector(ii) ~= previous_logic
                send_index = ii;
                previous_logic = true;
            end

            absolute_deviation = abs(actual_vector(ii)-previous_value);

            if absolute_deviation > deviation_threshold
                % only determine latency once per pulse. Once it has been
                % found, lock calculation until the pulse is low again
                if ~already_calculated_flag
                    receive_index  = ii;
                    latency_samples = receive_index - send_index
                    already_calculated_flag = true;
                end
            end
        else
            previous_logic = false;
            already_calculated_flag = false;
        end
        previous_value = actual_vector(ii);
    end
end