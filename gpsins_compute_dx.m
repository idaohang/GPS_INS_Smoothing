% This function computes error state
function [dx] = gpsins_compute_dx(p, xhat, xhat_1)

dR_b2t = xhat_1.R_b2t'*xhat.R_b2t;

dx = zeros(p.X_STATES, 1);
dx(p.X_ANG) = [-dR_b2t(2,3); dR_b2t(1,3); -dR_b2t(1,2)];
dx(p.X_VEL) = xhat.v_tb_t - xhat_1.v_tb_t;
dx(p.X_POS) = xhat.r_tb_t - xhat_1.r_tb_t;
dx(p.X_BA) = xhat.ba_b - xhat_1.ba_b;
dx(p.X_BG) = xhat.bg_b - xhat_1.bg_b;

