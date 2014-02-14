% This function provides dd meas of specific satellite prni w.r.t to common
% satellite prn0
function [Hi, PRange_i] = gpsins_sat_output(rpos_ecef, prn0, prni, meas_temp)
    H0 = zeros(1,3);
    svpos_ecef = meas_temp.Sat_state(prn0).sv_pos_ecef;
    H0(1) = svpos_ecef(1) - rpos_ecef(1);
    H0(2) = svpos_ecef(2) - rpos_ecef(2);
    H0(3) = svpos_ecef(3) - rpos_ecef(3);
    Rng_comp0 = norm( H0 );
    H0(1) = H0(1)/(-Rng_comp0);
    H0(2) = H0(2)/(-Rng_comp0);
    H0(3) = H0(3)/(-Rng_comp0);
    
    Hi = zeros(1,3);
    svpos_ecef = meas_temp.Sat_state(prni).sv_pos_ecef;
    Hi(1) = svpos_ecef(1) - rpos_ecef(1);
    Hi(2) = svpos_ecef(2) - rpos_ecef(2);
    Hi(3) = svpos_ecef(3) - rpos_ecef(3);
    Rng_comp = norm( Hi );
    Hi(1) = Hi(1)/(-Rng_comp)-H0(1);
    Hi(2) = Hi(2)/(-Rng_comp)-H0(2);
    Hi(3) = Hi(3)/(-Rng_comp)-H0(3);
    PRange_i = (Rng_comp - Rng_comp0);
end