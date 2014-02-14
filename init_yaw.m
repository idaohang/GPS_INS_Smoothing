% FIXME: sometimes this function does not work so well, you need to init
% yaw manually. 
% This function initialize yaw angle with doppler measurements, by
% considering the doppler meas as measurement of velocity. 
% The flow of this function is
% 1. Work on the first Sv in the prn list to find the first epoch has large
%    Doppler measurement
% 2. At this epoch, solve velocity estimate by Least Square with all Sv
%    Doppler
% 3. Init yaw angle with the resulted velocity.
function yaw = init_yaw(p, meas_temp, meas_temp_1)
%prn0 = data.gps(2).prnlist(1); % start from the second epoch
%D_temp = ( data.gps(2).Sat_state(prn0).rphase_l1 - data.gps(1).Sat_state(prn0).rphase_l1 ) * p.wave_l1
%( data.gps(2).Sat_state(prn0).doppler_l1 + data.gps(1).Sat_state(prn0).doppler_l1 )/2

% [rpos_ecef, delta_t] = LS_single_diff(meas_temp, p.ecef_p_b);
% 
% [rpos_ecef_1, delta_t_1] = LS_single_diff(meas_temp_1, rpos_ecef);

rpos_ecef = LS_double_diff(meas_temp, p.ecef_p_b, 1, 1);
rpos_ecef_1 = rpos_ecef;
delta_t = 0;
delta_t_1 = 0;

[num_meas, Meas, R_meas, prn_temp] = gps_calc_doppler(meas_temp);

H = zeros(num_meas, 4);
H_1 = zeros(num_meas, 4);

for i=1:num_meas
    prn = prn_temp(i);
    svpos_ecef = meas_temp.Sat_state(prn).sv_pos_ecef;
    H(i,1) = svpos_ecef(1) - rpos_ecef(1);
    H(i,2) = svpos_ecef(2) - rpos_ecef(2);
    H(i,3) = svpos_ecef(3) - rpos_ecef(3);
    Rng_comp(i) = norm( H(i,1:3) );
    H(i,1) = H(i,1)/(-Rng_comp(i));
    H(i,2) = H(i,2)/(-Rng_comp(i));
    H(i,3) = H(i,3)/(-Rng_comp(i));
    H(i,4) = 1;
    
    svpos_ecef_1 = meas_temp_1.Sat_state(prn).sv_pos_ecef;
    H_1(i,1) = svpos_ecef_1(1) - rpos_ecef_1(1);
    H_1(i,2) = svpos_ecef_1(2) - rpos_ecef_1(2);
    H_1(i,3) = svpos_ecef_1(3) - rpos_ecef_1(3);
    Rng_comp_1(i) = norm( H_1(i,1:3) );
end

residual = Meas* p.wave_l1 - ( Rng_comp - Rng_comp_1 )' - ( delta_t - delta_t_1 )*ones(num_meas,1);

Cov_meas = diag(1./R_meas);
vel_ecef = (H'*Cov_meas*H)\(H'*Cov_meas*residual);
vel = p.R_ned2ecef' * vel_ecef(1:3);
if norm(vel)<1 % if the rover moves slowly, skip init_yaw
    yaw = 0;
    return;
end
if vel(1)<0 && vel(1)>-1  % if vel_x is a very small negative number
    vel(1) = -vel(1);
end
yaw = limit_pi( atan(vel(2)/vel(1)) );
end