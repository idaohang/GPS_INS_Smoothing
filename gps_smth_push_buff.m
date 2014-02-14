% This function saves information from EKF for later use in smoothing
function [nav] = gps_smth_push_buff(gps_index, imu_index, nav)
nav.buff.kdim = nav.buff.kdim + 1;
kdim = nav.buff.kdim;
nav.buff.ydim = nav.buff.ydim + nav.dx.ydim;
nav.buff.index(kdim,:) = [gps_index, nav.buff.pre_imu_idx, imu_index];
nav.buff.pre_imu_idx = imu_index;
nav.buff.xhat_k(kdim,1) = nav.xhat;
nav.buff.Pxx_k = nav.dx.Pxx_minus;
end