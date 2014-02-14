% This function is to repropagate from the state xhat_i with imu meas. on
% interval confined by the index ij_imu
function [xhat_j, Phi_ji, Qd_j, Qd_i0] = gps_smth_iterate(p, xhat_i, Qd_i0, ij_imu, data)
Phi_ji = eye(p.X_STATES);
Qd_j = Qd_i0;
Qd_i0(:) = 0;

% % log data
% t = zeros(length(ij_imu)-1,1);
% pos = zeros(length(ij_imu)-1,3);
% t(1) = xhat_i.t;
% pos(1,:) = xhat_i.r_tb_t';

for ii_imu = (ij_imu(1)+1):ij_imu(2)
	% sample inertial sensor
	dt = data.imu(ii_imu).dt;
	ya_b = data.imu(ii_imu).accel_meas;
	yg_b = data.imu(ii_imu).gyro_meas;

	% propagate navigation state
	[xhat_i] = gpsins_propagate_xhat(p, xhat_i, ya_b, yg_b, dt);

	% compute system
	[Phi_i, Qd_i] = gpsins_compute_system(p, xhat_i, ya_b, yg_b, dt);

	% propagate Phi, Qd between aiding measurements
	Phi_ji = Phi_i*Phi_ji;
	Qd_j = Phi_i*Qd_j*Phi_i' + Qd_i;
    
%      % log data
%      t(ii_imu-ij_imu(1)+1) = xhat_i.t;
%      pos(ii_imu-ij_imu(1)+1,:) = xhat_i.r_tb_t';
    
end
xhat_j = xhat_i;

% plot propagation
% fig = 1000;
% h = figure(fig);
% subplot(3,1,1);
% plot(t, pos(:,1));
% title(['NED position'] );
% hold on;
% grid on;
% ylabel('x (m)');
% subplot(3,1,2);
% plot(t, pos(:,2));
% hold on;
% grid on;
% ylabel('y (m)');
% subplot(3,1,3);
% plot(t, pos(:,3));
% hold on;
% grid on;
% ylabel('z (m)');