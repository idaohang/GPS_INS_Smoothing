% This function returns the proper index of specific GPS data
function [ii] = gpsins_measurement_indices(gps, t_cpu, t_cpu_1)
j=1;
while gps(j).imu_tm <= t_cpu_1
    j=j+1;
end
if gps(j).imu_tm > t_cpu
    ii = 0;
else
    ii = j;
end

