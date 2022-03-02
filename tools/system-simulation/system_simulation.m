clc;
clear;
close all;

%% overview
% this script is for running a model of the car system to test various
% control algorithms for performance and feasability.

% % look at data to determine rough estimates for system time constant
% % load speed_data_vdigi_40_10_experiments.mat
% % load speed_data_vdigi_30_10_experiments.mat
% load speed_data_vdigi_35_10_experiments.mat
% 
% % indices for system response to vdigi
% indices_data_vdigi1 = detect_steps(IR_Speed_Right_Trackbased, 8);
% ir_speed_matrix_vdigi1 = [IR_Speed_Left_Trackbased(indices_data_vdigi1), IR_Speed_Right_Trackbased(indices_data_vdigi1)];
% 
% time_vector = Time(indices_data_vdigi1(:,1)) / 10^6;
% time_vector = time_vector - time_vector(1);
% ir_speed_vector_vdigi1 = plot_and_calculate_statistics(time_vector, ir_speed_matrix_vdigi1, 3);

%% model parameters and variables for system (PT1Tt element)
% for PT1, refer to https://de.wikipedia.org/wiki/PT1-Glied
% for Tt, refer to https://de.wikipedia.org/wiki/Totzeit_(Regelungstechnik)
simulation_sampling_time    = 1*10^-3; % in seconds. the faster, the more accurate but computationally more intensive
dead_time_maximum           = 100*10^-3; % in seconds. Maximum value for delay caused by clock desync between carrera control unit and sensorcar as well as wireless transmission delays.
system_time_constant        = 0.4;  % in seconds. tau or T1. System reaches 63.2% of asyptotic end value after T1. After 3*T1, approx. 95% is reached. 99.3% after 5*T1.
system_gain                 = 1;    % unitless.
available_speeds            = [0,0.607258, 0.926553, 1.302467, 1.72494, 1.696374, 2.129866, 2.522409, 2.918485, 3.296861, 3.64393, 3.943894, 4.25427, 4.536251, 4.73836 ]; % determined with experimentation, see speed_estimation_helper.m
available_vdigi             = [0, 20, 26, 32, 38, 44, 50, 56, 62, 68, 74, 80, 86, 92, 98];

%% track constants
track_straight              = 1;
track_curve_left_inner      = 2;
track_curve_left_outer      = 3;
track_curve_right_inner     = 4;
track_curve_right_outer     = 5;
trackpiece_length           = [345*10^-3, 259.181393921*10^-3, 362.85395149*10^-3, 259.181393921*10^-3, 362.85395149*10^-3]; % in meters. trackpiece_length[track_straight] is the length of the track_straight piece and so on
maximum_trackpiece_speed    = [4.6, 2.32954, 2.58, 2.32954, 2.58]; % in m/s. Same indexing as above. Values determined via experimentation. See data in derailing-data folder.

%% track parameters
% % small zero
% track_layout = [
%     track_straight, ...
%     track_curve_right_inner, track_curve_right_inner, track_curve_right_inner, ...
%     track_straight,... 
%     track_curve_right_inner, track_curve_right_inner, track_curve_right_inner
%     ];

% big zero "Titan"
track_layout = [
    track_straight, track_straight,...
    track_curve_right_inner, track_curve_right_inner, track_curve_right_inner, ...
    track_straight, track_straight, track_straight, ... 
    track_curve_right_inner, track_curve_right_inner, track_curve_right_inner, ...
    track_straight
    ];

% % parallelogram "Vanadium"
% track_layout = [
%     track_straight, track_straight, ...
%     track_curve_right_inner,...
%     track_straight,...
%     track_curve_right_inner,track_curve_right_inner,...
%     track_straight, track_straight, ...
%     track_curve_right_inner,...
%     track_straight,...
%     track_curve_right_inner,track_curve_right_inner
%     ];

% % hexagon "Chrome"
% track_layout = [
%     track_straight, track_curve_right_inner, ...
%     track_straight, track_curve_right_inner, ...
%     track_straight, track_curve_right_inner, ...
%     track_straight, track_curve_right_inner, ...
%     track_straight, track_curve_right_inner, ...
%     track_straight, track_curve_right_inner
%     ];

% average algorithm
% tries to drive the speed dictated by the average target speed of next number_of_pieces_to_average track pieces.


%% simulation and grid search parameters
total_simulation_laps = 5;  % amount of laps that should be driven until the simulation stops
speed_violation_tolerance_factor = 1.02;
stop_at_speed_violation = true; % stop simulation loop if speed limit was surpassed

do_grid_search = false;
do_plot = ~do_grid_search;            % output a plot with simulation data during run

target_speed_straight_grid_params       = available_speeds(1,2:end); % list of available speed settings.
target_speed_curve_grid_params          = available_speeds(1,2:end);
number_of_pieces_to_average_grid_params = 3:1:7; % list of parameters to test

weighting_average = true; % if false, values will be weighted randomly instead.

if ~do_grid_search
    % results from simulation
%     % optimized on titan or chrome: 5 laps - vanadium: failure - chrome: 1.5775 - titan: 1.5622
%     weights_vector = ones(1,3) / 3;
%     target_trackpiece_speed     = [2.5224, 2.1299, 2.1299, 2.1299, 2.1299]; % vdigi = [56, 50, ...]
%     % optimized on vanadium: 5 laps - vanadium: 1.5797 - chrome: 1.5775 - titan: 1.5810
%     weights_vector = ones(1,5) / 5;
%     target_trackpiece_speed     = [2.5224, 2.1299, 2.1299, 2.1299, 2.1299]; % vdigi = [56, 50, ...] 
    % real hardware
    weights_vector = ones(1,5) / 5;
    target_trackpiece_speed     = [2.9185, 1.6964, 1.6964, 1.6964, 1.6964]; % vdigi = [62, 44, ...]
end

%% grid search and simulation
if do_grid_search
    % vectors and matrices for logging results
    lap_time_mean_vector = NaN;
    lap_time_std_vector = NaN;
    target_straight_vector = NaN;
    target_curve_vector = NaN;
    number_of_pieces_to_average_vector = NaN;
    weights_vector_matrix = NaN(max(number_of_pieces_to_average_grid_params),1);

    for ii=1:1:length(target_speed_straight_grid_params)
        progress_percent = (ii) / length(target_speed_straight_grid_params) * 100
    for jj=1:1:length(target_speed_curve_grid_params)
    for nn=1:1:length(number_of_pieces_to_average_grid_params)
        if target_speed_straight_grid_params(ii) >= target_speed_curve_grid_params(jj) % only test if speed on straights is greater than in curves
            % set params for next simulation
            target_trackpiece_speed = [target_speed_straight_grid_params(ii), target_speed_curve_grid_params(jj), target_speed_curve_grid_params(jj), target_speed_curve_grid_params(jj), target_speed_curve_grid_params(jj)];
            number_of_pieces_to_average = number_of_pieces_to_average_grid_params(nn);
            if weighting_average
                weights_vector = ones(number_of_pieces_to_average, 1) / number_of_pieces_to_average;
            else
                weights_vector = rand(number_of_pieces_to_average, 1);
                weights_vector = weights_vector / sum(weights_vector); % normalize
            end

            [~, lap_time_mean, lap_time_standard] = simulate_system(do_plot, stop_at_speed_violation, simulation_sampling_time, dead_time_maximum, system_time_constant,system_gain,target_trackpiece_speed, maximum_trackpiece_speed, available_speeds, speed_violation_tolerance_factor, track_layout, weights_vector, total_simulation_laps);
    
            % data logging
            lap_time_mean_vector = [lap_time_mean_vector lap_time_mean];
            lap_time_std_vector = [lap_time_std_vector lap_time_standard];
            target_straight_vector = [target_straight_vector target_speed_straight_grid_params(ii)];
            target_curve_vector = [target_curve_vector target_speed_curve_grid_params(jj)];
            weights_vector_enhanced = zeros(max(number_of_pieces_to_average_grid_params),1); % pad weights with zeros
            weights_vector_enhanced(1:size(weights_vector)) = weights_vector;
            weights_vector_matrix = [weights_vector_matrix weights_vector_enhanced];
        end
    end
    end
    end
    % finding the best one
    %lap_time_mean_vector(lap_time_mean_vector==0) = NaN; % ignore unfinished races
    [minval, index_val] = min(lap_time_mean_vector);
    best_lap_time                       = minval
    best_value_straight_speed           = target_straight_vector(index_val)
    best_value_straight_vdigi           = available_vdigi(available_speeds==best_value_straight_speed)
    best_value_curve_speed              = target_curve_vector(index_val)
    best_value_curve_vdigi              = available_vdigi(available_speeds==best_value_curve_speed)
    best_weights                        = weights_vector_matrix(:,index_val);
    best_weights                        = best_weights(best_weights > 0) % only print nonzero values

    % simulate best one again
    target_trackpiece_speed     = [best_value_straight_speed, best_value_curve_speed, best_value_curve_speed, best_value_curve_speed, best_value_curve_speed];
    weights_vector              = best_weights;
    [lap_time_vector, lap_time_mean, lap_time_standard] = simulate_system(true, stop_at_speed_violation, simulation_sampling_time, dead_time_maximum, system_time_constant,system_gain,target_trackpiece_speed, maximum_trackpiece_speed, available_speeds, speed_violation_tolerance_factor, track_layout, weights_vector, total_simulation_laps)
else
    [lap_time_vector, lap_time_mean, lap_time_standard] = simulate_system(do_plot, stop_at_speed_violation, simulation_sampling_time, dead_time_maximum, system_time_constant,system_gain,target_trackpiece_speed, maximum_trackpiece_speed, available_speeds, speed_violation_tolerance_factor, track_layout, weights_vector, total_simulation_laps)
end

%% functions
% use this function to figure out the indices of where the measured speed
% should be sampled
function [] = evaluate_waveform(filename)
    load(filename,"-mat")
    figure();
    hold on
    stairs(IR_Speed_Left_Trackbased)
    stairs(IR_Speed_Right_Trackbased)    
    title('Click the plot to obtain an index.')
    xlabel('Index of sample')
    ylabel('Measured speed in m/s')
    legend('Left infrared sensor trackbased', 'Right infrared sensor trackbased')    
    clear;
    error('Execution halted. Run the script without this function to continue.')
end

% detect positive steps as new datapoint index and negative steps as
% beginning of a new dataset
function [indices_matrix] = detect_steps(input_vector, number_samples_per_dataset)
    deltas = zeros(size(input_vector));
    deltas(2:end) = (input_vector(1:end-1)-input_vector(2:end));
    indices_steps = find(deltas < 0);

    indices_steps(1:number_samples_per_dataset) = indices_steps(1:number_samples_per_dataset) + 1;
    indices_matrix = reshape(indices_steps, number_samples_per_dataset, []);

%     indices_datasets = [find(deltas > 0); length(input_vector)];
% 
%     for ii=1:1:length(indices_datasets)
%         if (ii > 1)
%             indices_matrix(:,ii) = indices_steps((indices_steps < indices_datasets(ii)) & (indices_steps > indices_datasets(ii-1)));
%         else
%             indices_matrix(:,ii) = indices_steps((indices_steps < indices_datasets(ii))) + 1;
%         end
%     end
end

function [median_vector] = plot_and_calculate_statistics(x_axis_vector, y_axis_data_matrix, number_of_sigmas)
    median_vector         = median(y_axis_data_matrix,2);
    standarddeviations    = std(y_axis_data_matrix,1,2);
    legendtext            = strcat("Median value with ", num2str(number_of_sigmas), " sigma error area");

    figure();
    hold on
    grid on
%     confplot(y_axis_vector, means, number_of_sigmas*standarddeviations)
    errorbar(x_axis_vector, median_vector, number_of_sigmas*standarddeviations)
    title('System response to step input , statistics')
    xlabel('Time in s')
    ylabel('Measured speed in m/s')
    legend(legendtext)
end

% total distance driven at the END of each track piece, as well as
% the total length of the track.
function [track_distances_vector, total_track_length] = calculate_total_track_distances(track_layout_vector, track_length, trackpiece_length_vector)
    sum_buffer = 0;
    track_distances_vector = zeros(size(track_layout_vector));
    for ii=1:1:track_length
        sum_buffer = sum_buffer + trackpiece_length_vector(track_layout_vector(ii)); % add the length of the current track piece
        track_distances_vector(ii) = sum_buffer;
    end
    total_track_length = sum_buffer;
end

function [remainder, division_success] = custom_modulo(dividend, divisor)
    times_it_fits       = floor(dividend/divisor);
    division_success    = (times_it_fits > 0);
    remainder           = dividend - times_it_fits * divisor;
end

function [incremented_variable] = increment_with_boundaries(variable, increment, maximum_value)
    incremented_variable = variable + increment;
    if incremented_variable > maximum_value
        incremented_variable = mod(incremented_variable, maximum_value);
    end
end

% main simulation function
function[lap_time_vector, lap_time_mean, lap_time_standard] = simulate_system(do_plot, stop_at_speed_violation, simulation_sampling_time, dead_time_maximum, system_time_constant,system_gain,target_trackpiece_speed, maximum_trackpiece_speed, available_speeds, speed_violation_tolerance_factor, track_layout, weights_vector, total_simulation_laps)
    % constants
    %system
    system_feedback_factor      = system_time_constant / (system_time_constant + simulation_sampling_time);
    system_input_factor         = system_gain * simulation_sampling_time / (system_time_constant + simulation_sampling_time);
    dead_time_samples           = ceil(dead_time_maximum/simulation_sampling_time);
    algorithm_clock_period = 75 * 10^-3; % in seconds, dicates update rate of target speed
    algorithm_clock_offset = 40 * 10^-3; % in seconds, random amount of offset that is <= algorithm_clock_period
    
    %track
    track_straight              = 1;
    track_curve_left_inner      = 2;
    track_curve_left_outer      = 3;
    track_curve_right_inner     = 4;
    track_curve_right_outer     = 5;
    trackpiece_length           = [345*10^-3, 259.181393921*10^-3, 362.85395149*10^-3, 259.181393921*10^-3, 362.85395149*10^-3]; % in meters. trackpiece_length[track_straight] is the length of the track_straight piece and so on
    track_length = length(track_layout); % in number of trackpieces
    [track_distance_checkpoints, total_track_length] = calculate_total_track_distances(track_layout, track_length, trackpiece_length); 
    
    % variables
    % timing variables
    sample_index        = dead_time_samples + 1;    % start value to avoid grabbing data from nonexistent indices
    starting_index      = sample_index;             % used to determine lap time
    stopping_index      = sample_index;             % used to determine lap time
    timestamp           = 0;                        % in s, time axis as well as a time to derive the algorithm clock from
    timestamp_vector    = zeros(1,sample_index);    % in s, logs timestamp
    
    alorithm_clock_timer            = algorithm_clock_offset;
    algorithm_rising_edge           = 0;                       % 1 if there is a rising edge, else 0.
    algorithm_rising_edge_vector    = zeros(1,sample_index);   % logs algorithm_rising_edge
    
    % vehicle variables
    track_position          = 1;                        % position of the car within the track
    track_position_vector   = ones(1,sample_index);    % logs track_position
    count_simulation_laps   = 0;                        % lap counter
    lap_time_vector         = NaN(1,total_simulation_laps);
    target_speed            = 0;                        % in m/s, input to the system
    target_speed_vector     = zeros(1,sample_index);    % in m/s, for logging and buffering of past samples
    speed_limit             = 0;                        % in m/s, running conditions that should not be violated
    speed_limit_vector      = zeros(1,sample_index);    % in m/s
    vehicle_speed           = 0;                        % in m/s, system output
    vehicle_speed_vector    = zeros(1,sample_index);    % in m/s
    vehicle_distance        = trackpiece_length(track_straight)/2;  % in m, also system output as per integration. Starting position is in the middle of a straight.
    vehicle_distance_vector = vehicle_distance * ones(1,sample_index);    % in m

    % process
    vehicle_derailed = false;
    while count_simulation_laps < total_simulation_laps
        % increment time by one timestep
        sample_index = sample_index + 1;
        timestamp = timestamp + simulation_sampling_time;
        timestamp_vector = [timestamp_vector, timestamp];
        
        % get algorithm clock rising edge event. This dictates when a new
        % target speed can be set.
        alorithm_clock_timer = alorithm_clock_timer + simulation_sampling_time;
        if ((alorithm_clock_timer / algorithm_clock_period) >= 1)
            alorithm_clock_timer = 0;
            algorithm_rising_edge = 1;
        else
            algorithm_rising_edge = 0;
        end
        algorithm_rising_edge_vector = [algorithm_rising_edge_vector, algorithm_rising_edge];
    
        % next number_of_pieces_to_average pieces algorithm for determining 
        % target speed
        if (algorithm_rising_edge == 1)
            sumbuf = 0;
            for ii=1:1:length(weights_vector)
                sumbuf = sumbuf + weights_vector(ii) * target_trackpiece_speed( track_layout( increment_with_boundaries(track_position,ii,track_length) ) );
            end
            % only possible speed settings on real hardware should be chosen.
            [~, minimum_index] = min(abs(available_speeds - sumbuf)); 
            target_speed = available_speeds(minimum_index);
        end
        target_speed_vector = [target_speed_vector, target_speed];
    
        speed_limit = maximum_trackpiece_speed( track_layout( track_position ));
        speed_limit_vector = [speed_limit_vector, speed_limit];
    
        % get vehicle speed based on system input and state
        % system difference equation (PT1Tt):
        % y(k) = T1/(T1+dt) * y(k-1) + K_PT1 * dt/(T1+dt) * x(k-1-Sd)
        % with y(k): output, x(k): input, Sd: delay in samples, dt: timestep,
        % T1: system time constant
        vehicle_speed = ...
            system_feedback_factor * vehicle_speed_vector(sample_index-1) + ...
            system_input_factor * target_speed_vector(sample_index-dead_time_samples);
        vehicle_speed_vector = [vehicle_speed_vector, vehicle_speed];
    
        % get vehicle position along the track by integrating speed. Trapezoid
        % formula is used for better accuracy.
        % y(k) = y(k-1) + (x(k)-x(k-1)) / 2 * dt
        % with y(k): output, x(k): input, dt: timestep
        vehicle_distance = ...
            vehicle_distance_vector(sample_index - 1) + ...
            (vehicle_speed_vector(sample_index) + ...
            vehicle_speed_vector(sample_index - 1)) / 2 * simulation_sampling_time;
        vehicle_distance_vector = [vehicle_distance_vector, vehicle_distance];
    
        % determine if a lap was driven, wrap around the total distance
        % travelled
        [leftover_track_driven, lap_completed] = custom_modulo(vehicle_distance_vector(sample_index), total_track_length);
        if lap_completed
            % determine lap time
            stopping_index = sample_index;
            lap_time_vector(count_simulation_laps + 1) = (stopping_index - starting_index) * simulation_sampling_time;
            starting_index = sample_index;
    
            track_position = 1;
            count_simulation_laps = count_simulation_laps + 1;
            % reset integration variables accordingly
            vehicle_distance_vector(sample_index) = leftover_track_driven;
            vehicle_distance_vector(sample_index-1) = leftover_track_driven;
        else
            if vehicle_distance >= track_distance_checkpoints(track_position)
                track_position = track_position + 1;    % will not overflow because it's reset every lap
            end
        end
        track_position_vector = [track_position_vector, track_position];

        if (vehicle_speed > speed_limit*speed_violation_tolerance_factor) && stop_at_speed_violation
            vehicle_derailed = true;
            break;
        end
    end
    
    %% plot results
    if do_plot
        figure();
        hold on
        grid on
        plot(timestamp_vector, vehicle_speed_vector ,'.-')
        % plot(timestamp_vector, vehicle_distance_vector)
        stairs(timestamp_vector, target_speed_vector, '.-')
        stairs(timestamp_vector, speed_limit_vector, '.-')
        stairs(timestamp_vector, track_position_vector / track_length, '.-')
        plot(timestamp_vector, algorithm_rising_edge_vector, '.-')
        title('Simulation of a race')
        xlabel('Time in seconds')
        ylabel('Speeds in m/s, others unitless')
        legend('Vehicle speed', 'Target speed', 'Speed limit', 'Vehicle position along the track', 'Rising edge of sampling clock')
    end
    
    % calculate the statistics
    if vehicle_derailed
        lap_time_mean = NaN;
        lap_time_standard = NaN;
    else
        lap_time_mean = mean(lap_time_vector(2:end));
        lap_time_standard = std(lap_time_vector(2:end),1);
    end
end