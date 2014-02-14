
% propagate navigation state (could utilize coning & sculling routines here)
function [xhat] = gpsins_propagate_xhat(p, xhat, ya_b, yg_b, dt)

% compute inertial quantities
omega_ib_b = yg_b - xhat.bg_b;
omega_tb_b = omega_ib_b - xhat.R_b2t'*p.omega_it_t;
f_ib_b = ya_b - xhat.ba_b;

% Euler integration
xhat.t = xhat.t + dt;
xhat.R_b2t = gpsins_correct_Rb2t(xhat.R_b2t, omega_tb_b*dt);
a_tb_t = xhat.R_b2t*f_ib_b + p.g_ib_t - 2*cross(p.omega_it_t, xhat.v_tb_t);
xhat.v_tb_t = xhat.v_tb_t + a_tb_t*dt;
xhat.r_tb_t = xhat.r_tb_t + xhat.v_tb_t*dt;

