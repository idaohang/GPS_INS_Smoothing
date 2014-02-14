% This function processes the navigation data for each IMU reading.
% If GPS meas received, apply EKF to update the estimates.
function [nav, out] = gps_smth_step(p, nav, ii_imu, data, out, Loose)
global DGPS;
global Dual_Freq;

nav.dx.applied = 0;
nav.dx.ydim = 0;

% sample inertial sensor
nav.ii_imu = ii_imu;
nav.t_cpu = data.imu(ii_imu).imu_tm;
if ii_imu > 1
	t_cpu_1 = data.imu(ii_imu-1).imu_tm;
else
	t_cpu_1 = data.imu(ii_imu).imu_tm - data.imu(ii_imu).dt; %0.0050;
end
dt = data.imu(ii_imu).dt;
ya_b = data.imu(max(ii_imu-1,1)).accel_meas;
yg_b = data.imu(max(ii_imu-1,1)).gyro_meas;
%  ya_b = data.imu(ii_imu).accel_meas;
%  yg_b = data.imu(ii_imu).gyro_meas;

% propagate navigation state
[nav.xhat] = gpsins_propagate_xhat(nav.param, nav.xhat, ya_b, yg_b, dt);

% propagate error state about trajectory
[nav.dx, Phi_i, Qd_i] = gpsins_propagate_dx(nav.param, nav.dx, nav.xhat, ya_b, yg_b, dt);


%sample GPS measurement
ii_gps = gpsins_measurement_indices(data.gps, nav.t_cpu, t_cpu_1);
if ii_gps>0
    pos_ecef = p.ecef_p_b + p.R_ned2ecef*nav.xhat.r_tb_t;
    if Loose
        [H, R, dy] = gpsins_gps_output_loose(nav.param, pos_ecef, data.gps(ii_gps), DGPS);
    else % tightly-couple
        [H, R, dy] = gpsins_gps_output(nav.param, pos_ecef, data.gps(ii_gps), DGPS, Dual_Freq);
    end
    % GPS aiding
    [nav.dx, out] = gpsins_correct_dx(nav.dx, H, R, dy, out);
end

% correct navigation state
if norm(nav.dx.dx_minus) > 0
	[nav.xhat] = gpsins_correct_xhat(nav.param, nav.xhat, nav.dx.dx_minus);
    nav.dx.dx_minus(:) = 0;
    if nav.dx.applied
        nav = gps_smth_push_buff(ii_gps,ii_imu, nav);
    end
end