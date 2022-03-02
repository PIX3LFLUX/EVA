clear;
clc;
close all;

%% this script accomplishes multiple things:
% - show accuracy of ir sensors left and right on a straight track
% - show how digital speed and measured speed are related
% - determine calibration values for the ir sensors on straights at
% moderate speeds (not used later since it's very inaccurate)

% obtain indices
% evaluate_waveform('speedsweep_data_fine1')

% % data, 16 samples
% file_base_name = 'speedsweep_data';
% indices_data(:,1) = [215; 495; 872; 1139; 1365; 1629; 1887; 2162; 2456; 2737; 3019; 3319; 3614; 3909; 4199; 4488];
% indices_data(:,2) = [222; 500; 862; 1133; 1390; 1700; 1913; 2200; 2475; 2765; 3047; 3338; 3631; 3917; 4212; 4501];
% indices_data(:,3) = [181; 497; 840; 1141; 1367; 1630; 1892; 2175; 2448; 2751; 3027; 3323; 3625; 3913; 4206; 4488];
% indices_data(:,4) = [110; 515; 863; 1137; 1418; 1621; 1894; 2177; 2457; 2747; 3039; 3331; 3627; 3911; 4239; 4487];
% indices_data(:,5) = [166; 527; 883; 1157; 1358; 1620; 1890; 2173; 2448; 2742; 3028; 3324; 3611; 3915; 4213; 4499];
% indices_data(:,6) = [110; 473; 849; 1074; 1324; 1598; 1879; 2155; 2437; 2732; 3014; 3325; 3601; 3907; 4265; 4461];

% data, 33 samples
file_base_name = 'speedsweep_data_fine';
indices_data(:,1) = [96; 760; 1766; 2231; 2940; 3512; 3970; 4568; 5098; 5701; 6425; 7001; 7573; 8049; 8743; 9331; 9834; 10444; 11149; 11637; 12224; 12834; 13566; 14222; 14623; 15450; 15999; 16598; 17181; 17631; 18348; 18956; 19718];

% load and plot data
[number_datapoints, number_files ]  = size(indices_data);
ir_speed_left_trackbased_matrix     = zeros(number_datapoints, number_files);
ir_speed_right_trackbased_matrix    = zeros(number_datapoints, number_files);
ir_speed_left_matrix                = zeros(number_datapoints, number_files);
ir_speed_right_matrix               = zeros(number_datapoints, number_files);
target_speed_vector                 = zeros(number_datapoints, 1);

figure();
for ii=1:1:number_files
    data_filename = strcat(file_base_name, num2str(ii),'.mat');
    load(data_filename,'-mat')
    plot_digital_vs_measured_speed(IR_Speed_Left_Trackbased, IR_Speed_Right_Trackbased, Target_Speed, indices_data(:,ii))

    ir_speed_left_trackbased_matrix(:,ii)   = IR_Speed_Left_Trackbased(indices_data(:,ii));
    ir_speed_right_trackbased_matrix(:,ii)  = IR_Speed_Right_Trackbased(indices_data(:,ii));
    ir_speed_left_matrix(:,ii)              = IR_Speed_Left(indices_data(:,ii));
    ir_speed_right_matrix(:,ii)             = IR_Speed_Right(indices_data(:,ii));    
    target_speed_vector                     = Target_Speed(indices_data(:,ii));
end

available_speeds = transpose(IR_Speed_Left_Trackbased(indices_data(1:2:29,1)))
available_vdigi  = transpose(Target_Speed(indices_data(1:2:29,1)))
available_vdigi  = [0, available_vdigi(2:end)]                          % lowest speed should be 0 to not burn out the motor

% indices_to_use = (target_speed_vector >= 20);   % only use a subset of data to do linear regression on. This only uses moderate speed, as errors are more prominent here
% [ m_left, b_left, calibrated_left ]  = determine_ir_calibration_params(ir_speed_left_matrix, ir_speed_left_trackbased_matrix, indices_to_use);
% [ m_right, b_right, calibrated_right]  = determine_ir_calibration_params(ir_speed_right_matrix, ir_speed_right_trackbased_matrix, indices_to_use);

% figure();
% plot_digital_vs_measured_speed_statistics(ir_speed_left_trackbased_matrix, target_speed_vector, 3)
% plot_digital_vs_measured_speed_statistics(ir_speed_right_trackbased_matrix, target_speed_vector, 3)
% plot_digital_vs_measured_speed_statistics(calibrated_left, target_speed_vector, 3)
% plot_digital_vs_measured_speed_statistics(calibrated_right, target_speed_vector, 3)

% use this function to figure out the indices of where the measured speed
% should be sampled
function [] = evaluate_waveform(filename)
    load(filename,"-mat")
    figure();
    hold on
    stairs(Target_Speed/20)
    stairs(IR_Speed_Left)
    stairs(IR_Speed_Right)
    stairs(IR_Speed_Left_Trackbased)
    stairs(IR_Speed_Right_Trackbased)    
    title('Click the plot to obtain an index.')
    xlabel('Index of sample')
    ylabel('Measured speed in m/s')
    legend('Target Speed / 20 in counts','Left infrared sensor', 'Right infrared sensor', 'Left infrared sensor trackbased', 'Right infrared sensor trackbased')    
    clear;
    error('Execution halted. Run the script without this function to continue.')
end

function [] = plot_digital_vs_measured_speed_statistics(x_axis_data_matrix, y_axis_vector, number_of_sigmas)
    means                 = median(x_axis_data_matrix,2);
    standarddeviations    = std(x_axis_data_matrix,1,2);
    legendtext            = strcat("Median value with ", num2str(number_of_sigmas), " sigma error area");

    hold on
    grid on
%     confplot(y_axis_vector, means, number_of_sigmas*standarddeviations)
    errorbar(y_axis_vector, means, number_of_sigmas*standarddeviations)
    title('Measured end speed of the car vs. digital speed value, statistics')
    xlabel('Digital speed in counts')
    ylabel('Measured speed in m/s')
    legend(legendtext)
end

function[m, b, approximated_data_matrix] = determine_ir_calibration_params(data_to_fit_matrix, reference_matrix, indices)
    data_means = median(data_to_fit_matrix,2);
    data_means = data_means(indices);
    ref_means = median(reference_matrix,2);
    ref_means = ref_means(indices);

    [ params ] = polyfit(data_means,ref_means, 1);
    m = params(1);
    b = params(2);
    approximated_data_matrix = m * data_to_fit_matrix + b;
    approximated_data = m * data_means + b;
    meansquareerror = sum((ref_means - approximated_data).*(ref_means - approximated_data)) / length(ref_means);
    
    figure();
    hold on
    grid on
    plot(data_means,ref_means)
    plot(data_means,approximated_data)
    title(strcat("Linear regression, MSE=", num2str(meansquareerror)))
    xlabel('Raw Data (x) in m/s')
    ylabel('Reference Data (y) in m/s')
    legend('Data vs. Reference', strcat("Regession Polynomial with y = m \cdot x + b, m=",num2str(m), ", b=", num2str(b)))
end

% this function obtains the differences of elements to their neighbor
function[diffed_vector] = get_differences(input_vector)
    diffed_vector = zeros(size(input_vector));
    diffed_vector(1:end-1) = input_vector(2:end)-input_vector(1:end-1);
end

function [] = plot_digital_vs_measured_speed(ir_speed_left, ir_speed_right, target_speed, indices)
    relative_difference = 100*(ir_speed_left(indices)-ir_speed_right(indices))./(ir_speed_right(indices)+10^-10);

    hold on
    grid on
    stairs(target_speed(indices), ir_speed_left(indices), 'o-')
    stairs(target_speed(indices), ir_speed_right(indices), 'o-')
    stairs(target_speed(indices), relative_difference, 'o-')
    title('Measured end speed of the car vs. digital speed value')
    xlabel('Digital speed in counts')
    ylabel('Measured speed in m/s')
    legend('Left infrared sensor', 'Right infrared sensor', 'Relative deviation of left sensor to right sensor in %')
end
