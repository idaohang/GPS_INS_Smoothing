% This function correct the rotation matrix with varphi.
% If varphi over some limit, error reported
function [R_plus,err_flag] = gpsins_correct_Rb2t(R_minus, varphi)
I3 = eye(3);
if dot(varphi,varphi) < 1

	% 1st order approximation
	R_plus = R_minus * (I3 + vcross(varphi));

	% normalize
	R_plus = (I3 - (1/2)*(R_plus*R_plus' - I3))*R_plus;
    err_flag = 1;
else
	fprintf(1, 'invalid Rb2t correction |varphi|=%g\n', norm(varphi));
	R_plus = R_minus;
    err_flag = 0;
end
