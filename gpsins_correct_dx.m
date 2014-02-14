% This function applies EKF
function [dx, out] = gpsins_correct_dx(dx, H, R, dy, out)
if ~isempty(dy)
	I = eye(size(dx.Pxx_minus));
	HPHtR = H*dx.Pxx_minus*H' + R;
	K = dx.Pxx_minus*H'*inv(HPHtR);
	dx_plus = K*(dy - H*dx.dx_minus);
	dx.dx_minus = dx.dx_minus + dx_plus;
	dx.Pxx_minus = (I - K*H)*dx.Pxx_minus*(I - K*H)' + K*R*K';
	dx.applied = 1;
	dx.ydim = dx.ydim + length(dy);
    
    % output residuals for analysis
%     if ~isempty(out)
%         if isfield(out, 'residual') && isfield(out.residual, 't')
%             i = length(out.residual.t) + 1;
%         else
%             i = 1;
%         end
%         out.residual.t(i,:) = dx.t';
%         out.residual.dy(i,:) = dy';
%         out.residual.HPHtR(i,:) = diag(HPHtR)';
%         out.residual.dx_plus(i,:) = dx_plus';
%     end
end
