% This function correct the estimate xhat based on dx_plus.
function [xhat, err_flag] = gpsins_correct_xhat(p, xhat, dx_plus)

% attitude error state
[xhat.R_b2t,err_flag] = gpsins_correct_Rb2t(xhat.R_b2t, dx_plus(p.X_ANG));

% velocity error state
xhat.v_tb_t = xhat.v_tb_t + dx_plus(p.X_VEL);

% position error state
xhat.r_tb_t = xhat.r_tb_t + dx_plus(p.X_POS);

% accelerometer bias error state
xhat.ba_b = xhat.ba_b + dx_plus(p.X_BA);

% gyro bias error state
xhat.bg_b = xhat.bg_b + dx_plus(p.X_BG);
