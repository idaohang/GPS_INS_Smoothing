
function [r, J, g, gps_res] = gps_smth_residual_jacobian(nav, Xhat_k, data, prn_list)
global DGPS;
global Dual_Freq;
%DGPS = 0;
Loose = 0;
Int_Free = 1;
% dimensions
N = nav.param.X_STATES;
K = nav.buff.kdim;
Y = nav.buff.ydim;

% initialize residual r(Xhat)
SigmaP = chol(inv(nav.buff.Pxx_k0));
rP = SigmaP*gpsins_compute_dx(nav.param, Xhat_k(1), nav.xhat0); % the last input changed from nav.xhat0
rQ = zeros(N*K, 1);
rY = zeros(Y, 1);

%fprintf(1, 'P_0(7,7)=%-12g P_0(8,8)=%-12g P_0(9,9)=%g\n', Pxx_k0(7,7), Pxx_k0(8,8), Pxx_k0(9,9));

% initialize Jacobian J(Xhat)
JP = [SigmaP zeros(N, K*N)];
JQ = zeros(N*K, N*(K+1));
JY = zeros(Y, N*(K+1));

gps_res.t = zeros(size(nav.buff.index,1),1);
gps_res.r = cell(size(nav.buff.index,1),1);
y = 1;
for k = 1:size(nav.buff.index,1)
    xhat_k_1 = Xhat_k(k);
    xhat_k = Xhat_k(k+1);
    
    ii_gps = nav.buff.index(k,1);
    % sample GPS measurements
    pos_ecef = nav.param.ecef_p_b + nav.param.R_ned2ecef*xhat_k.r_tb_t;
    if Loose
        [H, R, dy] = gpsins_gps_output_loose(nav.param, pos_ecef, data.gps(ii_gps), DGPS);
    else % tightly-coupling
        [H, R, dy] = gpsins_gps_output(nav.param, pos_ecef, data.gps(ii_gps), DGPS, Dual_Freq);
    end
%     if k==10
%         R=100*R;
%     end
    [rY, JY, y] = accumulateR(rY, JY, H, R, dy, y, k); %k
    gps_res.t(k,1) = xhat_k.t;
    gps_res.r{k} = (-chol(inv(R))*dy)';
    
    % repropagate and accumulate Q
    [xbar_k, Phi_k, Qd_k, nav.buff.Qd_k0] = gps_smth_iterate(nav.param, xhat_k_1, nav.buff.Qd_i0, [nav.buff.index(k,2),nav.buff.index(k,3)], data);
    [rQ, JQ] = accumulateQ(nav.param, rQ, JQ, xbar_k, xhat_k, Phi_k, Qd_k, k);
    
    %fprintf(1, '\nQ%g(7,7)=%-12g, Q%g(8,8)=%-12g, Q%g(9,9)=%g\n', k, Qd_k(7,7), k,Qd_k(8,8), k,Qd_k(9,9));
end

% check dimensions
if k ~= K
    error('invalid k-steps')
end
if y-1 ~= Y
    error('invalid measurement dimensions');
end

% output
r = [rP; rQ; rY];
J = [JP; JQ; JY];
% r = [rP; rQ];
% J = [JP; JQ];
if Int_Free
    Pos_ecef = zeros(K*3,1);
    for n=1:K
        Pos_ecef(3*n-2:3*n) = nav.param.ecef_p_b + nav.param.R_ned2ecef*Xhat_k(n+1).r_tb_t;
    end
    [H, R, dy] = int_free_phase_output(nav.param, Pos_ecef, data.gps(ii_gps-K+1:ii_gps), K, prn_list);
    if ~isempty(dy)
       SigmaR = chol(inv(R));
       rD = -SigmaR*dy;
       [a,b] = size(H);
       JD = [zeros(a,N),SigmaR*H]; % no x0
    end
    r = [r; rD];
    J = [J; JD];
end

g = J'*r;



function [rQ, JQ] = accumulateQ(p, rQ, JQ, xbar_k, xhat_k, Phi_k, Qd_k, k)
N = size(Phi_k, 1);
SigmaQ = chol(inv(Qd_k));
a_k = -gpsins_compute_dx(p, xhat_k, xbar_k);
rQ(((k-1)*N+1):(k*N), 1) = SigmaQ*a_k;
JQ(((k-1)*N+1):(k*N), ((k-1)*N+1):(k*N)) = SigmaQ*Phi_k;
JQ(((k-1)*N+1):(k*N), (k*N+1):((k+1)*N)) = -SigmaQ;


function [rY, JY, y] = accumulateR(rY, JY, H, R, dy, y, k)
N = size(H, 2);
m = size(dy, 1);
if ~isempty(dy)
    SigmaR = chol(inv(R));
    rY(y:(y+m-1), 1) = -SigmaR*dy;
    JY(y:(y+m-1), (k*N+1):((k+1)*N)) = SigmaR*H;
    y = y + m;
end


