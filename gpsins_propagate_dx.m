
% propagate error state
function [dx, Phi, Qd] = gpsins_propagate_dx(p, dx, xhat, ya_b, yg_b, dt)
if dt > 0
	[Phi, Qd] = gpsins_compute_system(p, xhat, ya_b, yg_b, dt);

	dx.dx_minus = Phi*dx.dx_minus;
	dx.Pxx_minus = Phi*dx.Pxx_minus*Phi' + Qd;
else
	Phi = eye(p.X_STATES);
	Qd = zeros(p.X_STATES);
end
dx.t = xhat.t;
