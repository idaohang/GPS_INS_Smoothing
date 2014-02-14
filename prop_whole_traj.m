% This function is used to propagate the whole state trajectory with X_hat
% and IMU data
function out = prop_whole_traj(nav, data, X_hat);
p = nav.param;

% number of IMU samples to process
N = size(data.imu,1);

% allocate memory for data analysis
out.t = ones(N,1)*NaN;
out.r_tb_t = ones(N,3)*NaN;
out.v_tb_t = ones(N,3)*NaN;
out.v_tb_b = ones(N,3)*NaN;
out.ba_b = ones(N,3)*NaN;
out.bg_b = ones(N,3)*NaN;
out.alphahat = ones(N,3)*NaN;
out.Pxx = ones(N,p.X_STATES)*NaN;

xhat = X_hat(1);
K = size(X_hat) - 1;
k = 1;
for ii_imu = 1:N    
    % sample inertial sensor
    t_cpu = data.imu(ii_imu).imu_tm;
    if ii_imu > 1
        t_cpu_1 = data.imu(ii_imu-1).imu_tm;
    else
        t_cpu_1 = data.imu(ii_imu).imu_tm - data.imu(ii_imu).dt; %0.0050;
    end
    dt = data.imu(ii_imu).dt;
    ya_b = data.imu(max(ii_imu-1,1)).accel_meas;
    yg_b = data.imu(max(ii_imu-1,1)).gyro_meas;
    
    % propagate navigation state
    [xhat] = gpsins_propagate_xhat(p, xhat, ya_b, yg_b, dt);
    
    %sample GPS measurement
    ii_gps = gpsins_measurement_indices(data.gps, t_cpu, t_cpu_1);
    if ii_gps>0
        k = k + 1;
        xhat = X_hat(k);
    end
    
    % log data
    out.t(ii_imu,:) = xhat.t;
    out.r_tb_t(ii_imu,:) = (xhat.r_tb_t)';
    out.v_tb_t(ii_imu,:) = (xhat.v_tb_t)';
    out.v_tb_b(ii_imu,:) = (xhat.R_b2t'*xhat.v_tb_t)';
    out.R_b2t(:,:,ii_imu)  = xhat.R_b2t;
    out.ba_b(ii_imu,:) = xhat.ba_b';
    out.bg_b(ii_imu,:) = xhat.bg_b';
    %out.Pxx(ii_imu,:) = diag(dx.Pxx_minus);
    xt = xhat.R_b2t*[1;0;0];
    out.alphahat(ii_imu,:) = atan2(xt(2),xt(1));
end