
function [nav] = gps_smth_buff_init(nav, buff_size)
nav.buff.kdim = 0;      % number of state
nav.buff.ydim = 0;      % total number of measurements
nav.buff.index = zeros(buff_size,3);
nav.buff.pre_imu_idx = 0;

nav.buff.xhat_k0 = nav.xhat;
nav.buff.xhat_k(buff_size,1) = nav.xhat;
nav.buff.xhat_k(1) = nav.xhat;

nav.buff.Pxx_k0 = nav.dx.Pxx_minus;
nav.buff.Qd_i0 = zeros(nav.param.X_STATES);
end
