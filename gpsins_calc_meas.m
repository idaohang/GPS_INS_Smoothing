% This function is for calculating double differenced measurements and
% covariance matrices.
% First, pick the common sv for double difference based on lockflag and
% baseflag;
% Second, calculate the double differenced meas based on availability of
% integer, phase, base, and so on.
function [num_meas, num_sats, Meas, R_meas, prn_dd] = gpsins_calc_meas(meas_temp, DGPS_ONLY, dual_freq)
if nargin<2
    DGPS_ONLY = 0;
end
num_meas = 0;
num_sats = 0;
if ~dual_freq
    Meas_temp = zeros(meas_temp.num_sats,1);
    R_meas_temp = zeros(meas_temp.num_sats,1);
else
    Meas_temp = zeros(meas_temp.num_sats,2);
    R_meas_temp = zeros(meas_temp.num_sats,2);
end

prn_dd_temp = zeros(meas_temp.num_sats,2);

if meas_temp.num_locked>0              % search common sat, do not need to search cd cm sat seperately
    for i=1:meas_temp.num_sats
        prn = meas_temp.prnlist(i);
        if meas_temp.Sat_state(prn).sat_valid
            if meas_temp.Sat_state(prn).locked % if phase common satellite found and this locked
                com_prn = prn;      % set ph common sat
                break;
            end
        end
    end
else if meas_temp.num_dgps>0
        for i=1:meas_temp.num_sats
            prn = meas_temp.prnlist(i);
            if meas_temp.Sat_state(prn).sat_valid
                if (meas_temp.Sat_state(prn).dgps_age >=0 && meas_temp.Sat_state(prn).dgps_age <=5) % if code common satellite not found and this dgps
                    com_prn = prn;      % set ph common sat
                    break
                end
            end
        end
    end
end

if DGPS_ONLY
    for i=1:meas_temp.num_sats
        prn = meas_temp.prnlist(i);
        if meas_temp.Sat_state(prn).sat_valid
            if prn~=com_prn && (meas_temp.Sat_state(prn).dgps_age >=0 && meas_temp.Sat_state(prn).dgps_age <=5)
                num_sats = num_sats + 1;
                prn_dd_temp(num_sats,:) = [prn, com_prn];
                
                num_meas = num_meas+1;
                Meas_temp(num_sats,1) = meas_temp.Sat_state(prn).sd_code_l1 - meas_temp.Sat_state(com_prn).sd_code_l1;
                R_meas_temp(num_sats,1) = meas_temp.Sat_state(prn).R_cd + meas_temp.Sat_state(com_prn).R_cd;
                if dual_freq
                    num_meas = num_meas+1;
                    Meas_temp(num_sats,2) = meas_temp.Sat_state(prn).sd_code_l2 - meas_temp.Sat_state(com_prn).sd_code_l2;
                    R_meas_temp(num_sats,2) = meas_temp.Sat_state(prn).R_cd + meas_temp.Sat_state(com_prn).R_cd;
                end
                %disp(['At time ', num2str(meas_temp.imu_tm),' Sv ', num2str(prn), ' use dd_code_l1']);
            end % else, prn = com_prn or no dgps, do nothing
        else
            %disp(['At time ', num2str(meas_temp.imu_tm),' for Sv ', num2str(prn), ' no measurement used']);
        end
    end
else
    for i=1:meas_temp.num_sats
        prn = meas_temp.prnlist(i);
        if meas_temp.Sat_state(prn).sat_valid
            if prn~=com_prn && meas_temp.Sat_state(prn).locked
                num_sats = num_sats + 1;
                prn_dd_temp(num_sats,:) = [prn, com_prn];
                
                num_meas = num_meas+1;
                Meas_temp(num_sats,1) = meas_temp.Sat_state(prn).ph_l1_rng - meas_temp.Sat_state(com_prn).ph_l1_rng;
                R_meas_temp(num_sats,1) = meas_temp.Sat_state(prn).R_ph + meas_temp.Sat_state(com_prn).R_ph;
                if dual_freq
                    num_meas = num_meas+1;
                    Meas_temp(num_sats,2) = meas_temp.Sat_state(prn).ph_l2_rng - meas_temp.Sat_state(com_prn).ph_l2_rng;
                    R_meas_temp(num_sats,2) = meas_temp.Sat_state(prn).R_ph + meas_temp.Sat_state(com_prn).R_ph;
                end
            else if ~meas_temp.Sat_state(prn).locked && (meas_temp.Sat_state(prn).dgps_age >=0 && meas_temp.Sat_state(prn).dgps_age <=5)
                    num_sats = num_sats + 1;
                    prn_dd_temp(num_sats,:) = [prn, com_prn];
                    
                    num_meas = num_meas+1;
                    Meas_temp(num_sats,1) = meas_temp.Sat_state(prn).sd_code_l1 - meas_temp.Sat_state(com_prn).sd_code_l1;
                    R_meas_temp(num_sats,1) = meas_temp.Sat_state(prn).R_cd + meas_temp.Sat_state(com_prn).R_cd;
                    if dual_freq
                        num_meas = num_meas+1;
                        Meas_temp(num_sats,2) = meas_temp.Sat_state(prn).sd_code_l2 - meas_temp.Sat_state(com_prn).sd_code_l2;
                        R_meas_temp(num_sats,2) = meas_temp.Sat_state(prn).R_cd + meas_temp.Sat_state(com_prn).R_cd;
                    end
                    %disp(['At time ', num2str(meas_temp.imu_tm),' Sv ', num2str(prn), ' use dd_code_l1']);
                end % else, prn = com_prn or no dgps, do nothing
            end
        else
            %disp(['At time ', num2str(meas_temp.imu_tm),' for Sv ', num2str(prn), ' no measurement used']);
        end
    end
end

if num_meas ~= 2*num_sats && dual_freq
    error('Meas number does not match Sats number!!!')
end

if dual_freq
    R_meas_temp = [ R_meas_temp(1:num_sats,1); R_meas_temp(1:num_sats,2)];
    Meas_temp   = [ Meas_temp(1:num_sats,1); Meas_temp(1:num_sats,2)];
else
    R_meas_temp = R_meas_temp(1:num_sats);
    Meas_temp   = Meas_temp(1:num_sats,1);
end

R_meas = diag(R_meas_temp);
prn_dd = prn_dd_temp(1:num_sats,:);
Meas = Meas_temp;