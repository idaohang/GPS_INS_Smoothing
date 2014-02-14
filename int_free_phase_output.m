% This function forms the residual and H for int_free_phase measurements. Check iROS paper.
function [H, R, residual] = int_free_phase_output(p, rpos_ecef, meas_buff, K, prn_list)
global Dual_Freq;
dual_freq = Dual_Freq;
k=1;
meas_temp = meas_buff(k);
[num_sat, Meas, R_meas, prn_dd] = int_free_calc_meas(meas_temp, prn_list); % get sat num to preallocate
%Meas = Meas*p.wave_l1;
H = zeros(num_sat*K,K*p.X_STATES); 

if ~dual_freq
    res_temp = zeros(num_sat*K,1);
else
    res_temp = zeros(num_sat*K,2);
end
R = zeros(num_sat*K,1);

V = zeros(num_sat*K,num_sat);
for i=1:num_sat
    V(K*(i-1)+1:K*i,i) = ones(K,1);
end
[Q, R_temp] = qr(V);
A = Q(:,num_sat+1:num_sat*K);

for j=1:K
    meas_temp = meas_buff(j); % for each measurement instant, from latest to oldest    
    [num_sat, sd_phase_meas, R_meas, prn_dd] = int_free_calc_meas(meas_temp, prn_list); % get the l1 phase in meters
    Meas = sd_phase_meas(1:num_sat)*p.wave_l1; % phase meas in meters
    if dual_freq
        Meas = [ Meas; ( sd_phase_meas(1:num_sat) - sd_phase_meas(1+num_sat:end) )*p.wave_wd];
    end
    for i=1:num_sat
        [Hij, PRangei] = gpsins_sat_output(rpos_ecef(3*j-2:3*j), prn_dd(i,2), prn_dd(i,1), meas_temp);
        H((i-1)*K+j,p.X_STATES*(j-1)+p.X_POS) = Hij*p.R_ned2ecef;
        res_temp((i-1)*K+j,1) = Meas(i) - PRangei ;%- Nhat(i)*p.wave_l1;
        R((i-1)*K+j) = R_meas(i,i);
        if dual_freq
            res_temp((i-1)*K+j,2) = Meas(i+num_sat) - PRangei;
        end
    end
end

residual = A'*res_temp(:,1);
if dual_freq
    H = [ A'*H; A'*H];
    residual = [ residual; A'*res_temp(:,2) ];
    R = blkdiag( A'*diag(R)*A, A'*diag(5.7^2*R)*A);  % 5.7 is from JAF2008 below eqn.8.106
else
    H = A'*H;
    R = A'*diag(R)*A;
end

end



% output residuals for analysis
% 	if ~isempty(out)
% 		if isfield(out, 'residual') & idx <= length(out.residual) & isfield(out.residual{idx}, 't')
% 			i = length(out.residual{idx}.t) + 1;
% 		else
% 			i = 1;
% 		end
% 		out.residual{idx}.t(i,:) = dx.t';
% 		out.residual{idx}.HPHtR(i,:) = diag(HPHtR)';
% 		out.residual{idx}.dy(i,:) = dy';
% 		out.residual{idx}.dx_plus(i,:) = dx_plus';
% 	end
