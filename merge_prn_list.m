function prn_list = merge_prn_list(data)
K = length(data.gps);
% init prn_list
num_sat = data.gps(1,1).num_sats; % FIXME: may use num_locked
prn_list = data.gps(1,1).prnlist(1:num_sat);
prn_list = clean_up_prn(prn_list, data.gps(1,1));

% Go through the data buffer and merge prn list
for i=2:K-1
    num_temp = data.gps(i,1).num_sats;
    prn_temp = data.gps(i,1).prnlist(1:num_temp);
    prn_temp = clean_up_prn(prn_temp, data.gps(i,1));
    prn_list = intersect(prn_list,prn_temp,'stable');
end
end

function list = clean_up_prn(list,meas)
global Elev_Mask;
elev_mask = Elev_Mask;
global Sat_Num;
sat_num = Sat_Num;

n = length(list);
bad_sats = zeros(n,1);
bad_num = 0;

% for i = 1:n
%     prn = list(i);
%     if ~meas.Sat_state(1,prn).locked
%         bad_num = bad_num + 1;
%         bad_sats(bad_num) = prn;
%     end
% end

for i = 1:n
    prn = list(i);
    if (meas.Sat_state(prn).dgps_age <0 || meas.Sat_state(prn).dgps_age >5 ...
            || meas.Sat_state(prn).sv_elev < elev_mask )
        bad_num = bad_num + 1;
        bad_sats(bad_num) = prn;
    end
end

for i = 1: bad_num   
    list = list( list~=bad_sats(i) );
end

if length(list) > sat_num
    list = list(1:sat_num,:);
end

end