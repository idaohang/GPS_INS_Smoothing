function [out, Xhat_k] = gps_smoothing(nav,data,prn_list)
p = nav.param;

% dimension of states
N = nav.param.X_STATES;

% number of states
K = nav.buff.kdim+1;

% dimension of correction
l = N*K;

% Reset Yaw and roll, pitch
if 1
    for j = 1:K-1        
        [roll, pitch, yaw] = dcm2angle( nav.buff.xhat_k(j).R_b2t', 'XYZ' );
        %R_t2b = angle2dcm(roll, pitch, yaw, 'XYZ');
        if j==1
            yaw = init_yaw(p, data.gps(2), data.gps(1));
        else
            yaw = init_yaw(p, data.gps(j), data.gps(j-1));
        end
        R_t2b = angle2dcm(0, 0, yaw, 'XYZ');
        %R_t2b = angle2dcm(0, 0, limit_pi(yaw - yaw0), 'XYZ');
        nav.buff.xhat_k(j).R_b2t = R_t2b';
    end
end

% initial conditions
Xhat_c = [nav.buff.xhat_k0;nav.buff.xhat_k];
Xhat_t = Xhat_c; % initialize temp Xhat_t

% plot_Xhat(nav, Xhat_c, K, 1);

itc = 1;
maxit = 100;
tol = 0.01;
alpha = 1.d-4;
[rc, Jc, gc, gps_res] = gps_smth_residual_jacobian(nav, Xhat_c, data, prn_list);
n_rc0 = norm(rc)
delta = 1;

%  plot_Xhat(nav, Xhat_t, K, itc);
% pause();

while (norm(delta)>tol && itc<=maxit)
    lambda = 1;
    % Gauss-Newton
    [Q, R] = qr(Jc);
    de = -Q'*rc;
    d = de(1:l, :);
    R = R(1:l, :);
    dc = R\d;
    %dc = -(Jc'*Jc)\(Jc'*rc);
    dgn = lambda*dc;
    %n_dgn = norm(dgn);
    delta = reshape(dgn, N, K)';
    
    for k = 1:size(Xhat_c,1)
        Xhat_t(k) = gpsins_correct_xhat(nav.param, Xhat_c(k), delta(k,:)');
    end
    n_rc0 = norm(rc)
    itc
    
    
%      plot_Xhat(nav, Xhat_t, K, itc+1);
%     pause()
    
    [rt, Jt, gt] = gps_smth_residual_jacobian(nav, Xhat_t, data, prn_list);
    iarm=0;
    itc=itc+1;
    %
    % Goal for sufficient decrease
    %    
    %rgoal = norm(rc) - alpha*lambda*(gc'*dc); % dgn=lambda*dc
    Armijo = 0;
    %while(norm(rt)>rgoal)
    while(norm(rt)>norm(rc))
        iarm=iarm+1;
        lambda=lambda/2;  % 1.2
        %rgoal = norm(rc) - alpha*lambda*(gc'*dc);
        dgn = lambda*dc;
        delta = reshape(dgn, N, K)';
        for k = 1:size(Xhat_c,1)
            Xhat_t(k) = gpsins_correct_xhat(nav.param, Xhat_c(k), delta(k,:)');
        end
        [rt, Jt, gt] = gps_smth_residual_jacobian(nav, Xhat_t, data, prn_list);
        if(iarm > 20)
            disp(' Armijo error in Gauss-Newton')
            Armijo = 1;
            break;
        end
    end
    if Armijo
        break;
    else
        Xhat_c = Xhat_t;
        [rc, Jc, gc, gps_res] = gps_smth_residual_jacobian(nav, Xhat_c, data, prn_list);
        norm(rc)
    end        
end

disp(['dX = ', num2str(norm(delta))]);
disp(['iteration ', num2str(itc)]);
Xhat_k = Xhat_c;
Xhat_t = Xhat_c;

for ii = 1:K
	% log data
	out.t(ii,:) = Xhat_k(ii).t;
	out.r_tb_t(ii,:) = (Xhat_k(ii).r_tb_t)';
	out.v_tb_t(ii,:) = (Xhat_k(ii).v_tb_t)';
	out.v_tb_b(ii,:) = (Xhat_k(ii).R_b2t'*Xhat_k(ii).v_tb_t)';
    out.R_b2t(:,:,ii)  = Xhat_k(ii).R_b2t;
	out.ba_b(ii,:) = Xhat_k(ii).ba_b';
	out.bg_b(ii,:) = Xhat_k(ii).bg_b';
	%out.Pxx(ii,:) = diag(nav.dx.Pxx_minus);
	xt = Xhat_k(ii).R_b2t*[1;0;0];
	out.alphahat(ii,:) = atan2(xt(2),xt(1));
    out.res = gps_res;
end
% 
% h = figure(1);
% subplot(3,1,1);
% plot(out.t, out.r_tb_t(:,1)-nav.param.base_pos(1),'--*r');
% title(['NED position'] );
% hold on;
% grid on;
% ylabel('x (m)');
% subplot(3,1,2);
% plot(out.t, out.r_tb_t(:,2)-nav.param.base_pos(2),'--*r');
% hold on;
% grid on;
% ylabel('y (m)');
% subplot(3,1,3);
% plot(out.t, out.r_tb_t(:,3)-nav.param.base_pos(3),'--*r');
% hold on;
% grid on;
% ylabel('z (m)');

end