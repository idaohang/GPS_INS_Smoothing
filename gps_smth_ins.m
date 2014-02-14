% This script is for validate gps-ins smoothing only. 
% No sliding window implemented.
function [X_hat, traj, smth_out, ekf_out] = gps_smth_ins(Start_time, Buff_Size, state_out)
%clc;
%clear;
%close all;

r2d = 180/pi;
global imu_data;
global gps_data;

%% Options
End_time = Start_time + Buff_Size;
LOOSE = 0;     % for loosely coupling in EKF
DGPS = 1;      % if 1, then only use differenced code in EKF, no int solved phase

%% Load data
data.imu = read_IMU_buff_log(imu_data, Start_time, End_time);
[~,m] = size(gps_data);
if m>1525
    data.gps = read_GPS_buff_log_new(gps_data, Start_time, End_time);
else
    data.gps = read_GPS_buff_log(gps_data, Start_time, End_time);
end

%% Simulation setup
% simulation parameters
p = gpsins_param();

% number of IMU samples to process
N = size(data.imu,1);

% allocate memory for data analysis
ekf_out.t = ones(N,1)*NaN;
ekf_out.r_tb_t = ones(N,3)*NaN;
ekf_out.v_tb_t = ones(N,3)*NaN;
ekf_out.v_tb_b = ones(N,3)*NaN;
ekf_out.ba_b = ones(N,3)*NaN;
ekf_out.bg_b = ones(N,3)*NaN;
ekf_out.alphahat = ones(N,3)*NaN;
ekf_out.Pxx = ones(N,p.X_STATES)*NaN;

% init
nav = gps_smth_init(data, p, Start_time);
nav = gps_smth_buff_init(nav, Buff_Size);

% use EKF result as initial estimates of smoothing later
for ii_imu = 1:N
    % perform one navigation iteration
    [nav, ekf_out] = gps_smth_step(p, nav, ii_imu, data, ekf_out, LOOSE);
    
    % log data
    ekf_out.t(ii_imu,:) = nav.xhat.t;
    ekf_out.r_tb_t(ii_imu,:) = (nav.xhat.r_tb_t)';
    ekf_out.v_tb_t(ii_imu,:) = (nav.xhat.v_tb_t)';
    ekf_out.v_tb_b(ii_imu,:) = (nav.xhat.R_b2t'*nav.xhat.v_tb_t)';
    ekf_out.R_b2t(:,:,ii_imu)  = nav.xhat.R_b2t;
    ekf_out.ba_b(ii_imu,:) = nav.xhat.ba_b';
    ekf_out.bg_b(ii_imu,:) = nav.xhat.bg_b';
    ekf_out.Pxx(ii_imu,:) = diag(nav.dx.Pxx_minus);
    xt = nav.xhat.R_b2t*[1;0;0];
    ekf_out.alphahat(ii_imu,:) = atan2(xt(2),xt(1));
    
    if find(isnan(nav.dx.Pxx_minus))
        break
    end
    
    if (nav.buff.kdim==Buff_Size)
        break
    end
end

% handle the prnlist here
prn_list = merge_prn_list(data);

Xhat_0 = [nav.buff.xhat_k0;nav.buff.xhat_k]; % save the initial estimates

% Do smoothing
[smth_out, X_hat] = gps_smoothing(nav, data, prn_list);

% Propagate whole trajectory
traj = prop_whole_traj(nav, data, X_hat);


