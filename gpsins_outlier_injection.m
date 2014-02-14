% This function is to inject outlier to imu_data and(or) gps_data
function [data] = eraim_outlier_injection(data)
time = round(length(data.gps)/2);
time = 1:length(data.gps);
noise_code = 25; % in meters
noise_phase = 25; % in cycles

prn = data.gps(time(1)).prnlist(3);

for i = 1:length(time);
    data.gps(time(i)).Sat_state(1,prn).sd_code_l1 = ...
        data.gps(time(i)).Sat_state(1,prn).sd_code_l1 + 25;
end


