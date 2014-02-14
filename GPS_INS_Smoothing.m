%% Data Sets
% 0119_1042: Veh_data_0613_1055
% 20140124_1728: Ahn's loop on the magazine paper
% 20140124_1848: 600s open sky around CECERT
% UCRStationary: two antennas on campus

clc;
clear;
close all;

global Dual_Freq;
global Stationary;
global Elev_Mask;
global Sat_Num;
Dual_Freq = 0;            % use dual freq meas or just single
Stationary = 1;           % work on stationary data or on vehicle data
Elev_Mask = 0;            % elevation mask in degree
Sat_Num = 16;             % max number of satellite to use

skipp = 25;                                      % skip the first xx seconds
buff_size = 10;                                 % smoothing buffer size

% for stationary
true = [0.025, -5.931, 0.051]; % True Displacement between Ant1 and Ant2

% Load data
global imu_data;
global gps_data;
if ~Stationary
    IMU_meas_log20140124_1728;
    %IMU_meas_log20140119_1042;
    imu_data = imu_meas;
    gps_data = load('./Data_set/GPS_buff_log20140124_1728.dat');
    %gps_data = load('./Data_set/GPS_buff_log20140119_1042.dat');
    gps_data(:,1) = gps_data(:,2); % fix time tag
    %load('./Data_set/python_smooth.mat'); % smoothing result in state_out
    load('./Data_set/Ahns_loop.mat'); % smoothing result in state_out
else
    imu_data = load('./Data_set/IMU_buff_log_UCRStationary_1000s.dat');
    gps_data = load('./Data_set/GPS_buff_log_UCRStationary_1000s.dat');
end

[K,~] = size(gps_data);

start_time = round( gps_data(skipp,1) );
if (start_time+buff_size)>gps_data(end,1);
    display('Reached the end.');
    break;
end

if ~Stationary
    % trim data for plotting
    i = 1;
    while ( state_out(i,1) < start_time )
        i=i+1;
    end
    j=i+1;
    while ( state_out(j,1)<start_time+buff_size)
        j=j+1;
    end
    state_out_k = state_out(i:j,:);
else
    state_out_k = [];
end

[Xhat, traj] = gps_smth_ins( start_time , buff_size, state_out_k );

m = size(state_out_k);
if m
    display('Sorry, due to time tag problem, no ground truth comparison in statistics for vehicle data temporarily.')
    figure(1);
    subplot(3,1,1);
    plot(state_out_k(:,1), state_out_k(:,2), '.k', traj.t, traj.r_tb_t(:,1), '.b' );
    title(['NED position'] );
    hold on;
    grid on;
    ylabel('x (m)');
    subplot(3,1,2);
    plot(state_out_k(:,1), state_out_k(:,3), '.k', traj.t, traj.r_tb_t(:,2), '.b' );
    hold on;
    grid on;
    ylabel('y (m)');
    subplot(3,1,3);
    plot(state_out_k(:,1), state_out_k(:,4), '.k', traj.t, traj.r_tb_t(:,3), '.b' );
    hold on;
    grid on;
    ylabel('z (m)');
    xlabel('time (sec)')
    
    figure(2);
    plot(state_out_k(:,3), state_out_k(:,2), '.k', traj.r_tb_t(:,2), traj.r_tb_t(:,1), '.b' );
    hold on;
    grid on;
    ylabel('North');
    xlabel('East');
else
    out11 = [traj.t([1:2:end],:), traj.r_tb_t([1:2:end],:)];
    one_vec = ones(length(out11),1);
    ground_truth = [true(1)*one_vec, true(2)*one_vec, true(3)*one_vec];
    error1 = mean( ground_truth - out11(:,2:4));
    var1 = var( ground_truth - out11(:,2:4));
    std1 = std( ground_truth - out11(:,2:4));
    
    display(['Pos error mean is ', num2str(error1)] );
    display(['Pos error variance is ', num2str(var1)] );
    display(['Pos error std_dev is ', num2str(std1)] );
end
