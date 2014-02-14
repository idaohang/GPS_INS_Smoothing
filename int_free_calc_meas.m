function [num_sats, Meas, R_meas, prn_dd] = int_free_calc_meas(meas_temp, prn_list)
global Dual_Freq;
dual_freq = Dual_Freq;
num_meas = 0;
num_sats = 0;
if ~dual_freq
    Meas_temp = zeros(meas_temp.num_sats,1);
else
    Meas_temp = zeros(meas_temp.num_sats,2);
end
R_meas_temp = zeros(meas_temp.num_sats,1);
prn_dd_temp = zeros(meas_temp.num_sats,2);


com_prn_ph = prn_list(1);      % set ph common sat

for i=2:length(prn_list)
    prn = prn_list(i);
    if prn~=com_prn_ph && (meas_temp.Sat_state(prn).dgps_age >=0 && meas_temp.Sat_state(prn).dgps_age <=5)
        num_sats = num_sats + 1;
        prn_dd_temp(num_sats,:) = [prn, com_prn_ph];
        num_meas = num_meas+1;
        Meas_temp(num_sats,1) = (meas_temp.Sat_state(prn).sd_phase_l1 - meas_temp.Sat_state(com_prn_ph).sd_phase_l1);
        R_meas_temp(num_sats,1) = meas_temp.Sat_state(prn).R_ph + meas_temp.Sat_state(com_prn_ph).R_ph;
        
        if Dual_Freq
            num_meas = num_meas+1;
            Meas_temp(num_sats,2) = (meas_temp.Sat_state(prn).sd_phase_l2 - meas_temp.Sat_state(com_prn_ph).sd_phase_l2);
        end
    else
        disp(['At time ', num2str(meas_temp.imu_tm),' for Sv ', num2str(prn), ' no measurement used']);
    end
end

if num_meas ~= 2*num_sats && dual_freq
    error('Meas number does not match Sats number!!!')
end

if dual_freq
    R_meas_temp = [ R_meas_temp(1:num_sats); R_meas_temp(1:num_sats)];
    Meas_temp   = [ Meas_temp(1:num_sats,1); Meas_temp(1:num_sats,2)];
else
    R_meas_temp = R_meas_temp(1:num_sats);
    Meas_temp   = Meas_temp(1:num_sats,1);
end

R_meas = diag(R_meas_temp);
prn_dd = prn_dd_temp(1:num_sats,:);
Meas = Meas_temp;