% This function provide GPS-only position fix
% Inputs: 
%   dgps: only differenced code used
%   dual_freq: dual_freq meas used
% Output: rover position in ECEF.
function [rpos_ecef] = LS_double_diff(meas_temp, init_pos_ecef, dgps, dual_freq)
%% options
DGPS = dgps;
CODE_ONLY = 0;
%% constants
GPS_TH = 0.0001;
%%
rpos_ecef = init_pos_ecef;
svpos_ecef = zeros(3,1);
delta = zeros(3,1);

[num_meas, num_sats, Meas, R_meas, prn_dd] = gpsins_calc_meas(meas_temp, DGPS, dual_freq);

if num_meas ~= 2*num_sats && dual_freq
    error('Meas number does not match Sats number!!!')
end

H0 = zeros(1,3);
H = zeros(num_meas,3);
Rng_comp = zeros(num_meas,1);
PRange = zeros(num_meas,1);
 
recur_count = 0;
while(recur_count==0 || norm(delta)> 0.01*GPS_TH)
    for i=1:num_sats
        prn0 = prn_dd(i,2);
        svpos_ecef = meas_temp.Sat_state(prn0).sv_pos_ecef;
        H0(1) = svpos_ecef(1) - rpos_ecef(1);
        H0(2) = svpos_ecef(2) - rpos_ecef(2);
        H0(3) = svpos_ecef(3) - rpos_ecef(3);
        Rng_comp0 = norm( H0 );
        H0(1) = H0(1)/(-Rng_comp0);
        H0(2) = H0(2)/(-Rng_comp0);
        H0(3) = H0(3)/(-Rng_comp0);
        
        prni = prn_dd(i,1);
        svpos_ecef = meas_temp.Sat_state(prni).sv_pos_ecef;
        H(i,1) = svpos_ecef(1) - rpos_ecef(1);
        H(i,2) = svpos_ecef(2) - rpos_ecef(2);
        H(i,3) = svpos_ecef(3) - rpos_ecef(3);
        Rng_comp(i) = norm( H(i,:) );
        H(i,1) = H(i,1)/(-Rng_comp(i))-H0(1);
        H(i,2) = H(i,2)/(-Rng_comp(i))-H0(2);
        H(i,3) = H(i,3)/(-Rng_comp(i))-H0(3);
        PRange(i) = (Rng_comp(i) - Rng_comp0);
        
        if dual_freq
            H(i+num_sats,:) = H(i,:);
            PRange(i+num_sats,:) = PRange(i,:);
        end
    end
    Cov_meas = inv(R_meas);
    residual = Meas - PRange;
    delta = (H'*Cov_meas*H)\(H'*Cov_meas*residual);
    rpos_ecef = rpos_ecef + delta;
    recur_count=recur_count+1;
end

norm(delta);
% recur_count
end
