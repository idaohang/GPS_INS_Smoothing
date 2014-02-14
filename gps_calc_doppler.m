% This function retrive doppler measurements from data struct
function [num_meas, Meas, R_meas, prn_temp] = gps_calc_doppler(meas_temp)

num_meas = 0;
Meas_temp = zeros(meas_temp.num_sats,1);
R_meas_temp = zeros(meas_temp.num_sats,1);
prn_temp = zeros(meas_temp.num_sats,1);

for i=1:meas_temp.num_sats
    prn = meas_temp.prnlist(i);
    if meas_temp.Sat_state(prn).sat_valid
        if meas_temp.Sat_state(prn).dgps_age >=0 && meas_temp.Sat_state(prn).dgps_age <=5
            num_meas = num_meas+1;
            prn_temp(num_meas,:) = prn;
            
            Meas_temp(num_meas) = meas_temp.Sat_state(prn).doppler_l1;
            R_meas_temp(num_meas) = meas_temp.Sat_state(prn).R_dplr;
        else
            disp(['At time ', num2str(meas_temp.imu_tm),' for Sv ', num2str(prn), ' no measurement used']);
        end
    else
        disp(['At time ', num2str(meas_temp.imu_tm),' for Sv ', num2str(prn), ' no measurement used. Since invalid.']);
    end
end

R_meas = R_meas_temp(1:num_meas);
prn_temp = prn_temp(1:num_meas,:);
Meas = Meas_temp(1:num_meas);