function [buffer] = read_GPS_buff_log(buff_log, start_time, end_time)
NUM_SATS = 15;
NUM_PRNS = 32;

if nargin < 2
	start_time = 0;
    end_time = 1000;
else if nargin < 3
        end_time = 1000;
    end
end

i = 1;
% push buffer from the starting time
while ( buff_log(i,1)<start_time )
    i=i+1;
end
j=i+1;
while ( buff_log(j,1)<end_time )
    j=j+1;
end
buff_log = buff_log(i:j,:);
buff_size = length( buff_log(:,1) );

for index = 1:buff_size
    data = buff_log(index,:);
    count = 1;
    meas_temp.imu_tm = data(count);
    count=count+1;
    meas_temp.pps_tm = data(count);
    count=count+1;
    meas_temp.tow = data(count);
    count=count+1;
    meas_temp.num_sats = data(count);
    count=count+1;
    meas_temp.num_locked = data(count);
    count=count+1;
    meas_temp.num_dgps = data(count);
    count=count+1;

    meas_temp.prnlist = zeros(meas_temp.num_sats,1);
    for i=1:NUM_SATS
        meas_temp.prnlist(i) = data(count);
        count=count+1;
    end
    for i=1:NUM_PRNS
        meas_temp.Sat_state(i).sat_valid = data(count);
        count=count+1;
        meas_temp.Sat_state(i).locked = data(count);
        count=count+1;
        meas_temp.Sat_state(i).prn_num = data(count);
        count=count+1;
        meas_temp.Sat_state(i).aode = data(count);
        count=count+1;
        meas_temp.Sat_state(i).dgps_age = data(count);
        count=count+1;
        meas_temp.Sat_state(i).N_l1 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).N_l2 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).N_nr = data(count);
        count=count+1;
        meas_temp.Sat_state(i).N_wd = data(count);
        count=count+1;
        meas_temp.Sat_state(i).ph_l1_rng = data(count);
        count=count+1;
        meas_temp.Sat_state(i).ph_l2_rng = data(count);
        count=count+1;
        meas_temp.Sat_state(i).ph_nr_rng = data(count);
        count=count+1;
        meas_temp.Sat_state(i).ph_wd_rng = data(count);
        count=count+1;
        meas_temp.Sat_state(i).rcode_l1 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).rcode_l2 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).rphase_l1 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).rphase_l2 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).doppler_l1 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).doppler_l2 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).std_code_l1 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).std_code_l2 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).std_phase_l1 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).std_phase_l2 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).d_tsv = data(count);
        count=count+1;
        meas_temp.Sat_state(i).std_phase_l1 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).std_phase_l2 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).code_l1 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).code_l2 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).phase_l1 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).phase_l2 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).corr_code_l1 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).corr_code_l2 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).corr_phase_l1 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).corr_phase_l2 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).sd_code_l1 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).sd_code_l2 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).sd_phase_l1 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).sd_phase_l2 = data(count);
        count=count+1;
        meas_temp.Sat_state(i).sv_pos_ecef(1) = data(count);
        count=count+1;
        meas_temp.Sat_state(i).sv_pos_ecef(2) = data(count);
        count=count+1;
        meas_temp.Sat_state(i).sv_pos_ecef(3) = data(count);
        count=count+1;
        meas_temp.Sat_state(i).sv_vel_ecef(1) = data(count);
        count=count+1;
        meas_temp.Sat_state(i).sv_vel_ecef(2) = data(count);
        count=count+1;
        meas_temp.Sat_state(i).sv_vel_ecef(3) = data(count);
        count=count+1;
        meas_temp.Sat_state(i).R_cd = data(count);
        count=count+1;
        meas_temp.Sat_state(i).R_ph = data(count);
        count=count+1;
        meas_temp.Sat_state(i).R_dplr = data(count);
        count=count+1;
        meas_temp.Sat_state(i).sv_elev = 90; % assume no elevation mask
    end
    if index == 1
        buffer(buff_size,1)=meas_temp;
        buffer(1)=meas_temp;
    else
        buffer(index)=meas_temp;
    end
end


