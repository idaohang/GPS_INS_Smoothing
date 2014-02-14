% Paul Miller
% limit_pi(): Limit angles between +/-pi
% $Id: limit_pi.m 4300 2010-01-05 22:28:58Z 02millerp88 $

function [x]=limit_pi(x)
i = find(x > pi);
if ~isempty(i),
    x(i) = x(i) - 2*pi*floor((x(i)+pi)/(2*pi));
end
i = find(x < -pi);
if ~isempty(i),
    x(i) = x(i) - 2*pi*ceil((x(i)-pi)/(2*pi));
end
