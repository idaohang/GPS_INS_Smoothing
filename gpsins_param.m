
function [p] = gpsins_param()
%% constants
r2d = 180/pi;
d2r = pi/180;
p.grav = 9.80665; % m/s/s
p.NUM_SATS = 15; 
p.NUM_PRNS = 32;

% Ant 1
p.R_ned2ecef = [0.2565316863   0.8884106506  0.3806809816
              0.4964724789  -0.4590495789  0.7367418556
              0.8292807556   0            -0.5588322005];

%p.ecef_p_b = [-2430696.646 -4704190.763 3544329.081]'; % in meters

% Ant 2
p.ecef_p_b = [-2430691.399 -4704193.471 3544329.054]';

p.base_pos = zeros(3,1);%[-0.024,5.928,-0.055]';%;
p.freq_l1  = 1575420000.0;               % in Hz
p.freq_l2  = 1227600000.0;               % in Hz
p.lightspeed = 2.99792458e8; % m/s           
p.wave_l1 = (p.lightspeed / p.freq_l1); % meters
p.wave_l2 = (p.lightspeed / p.freq_l2); % meters
p.wave_wd = ( p.lightspeed / ( p.freq_l1 - p.freq_l2 ) ); % meters
p.wave_nr = ( p.lightspeed / ( p.freq_l1 + p.freq_l2 ) ); % meters

% state indices
p.X_STATES = 15;
p.X_ANG = 1:3;
p.X_VEL = 4:6;
p.X_POS = 7:9;
p.X_BA = 10:12;
p.X_BG = 13:15;

p.X_names(p.X_ANG) = {'tilt-1','tilt-2','tilt-3'};
p.X_units(p.X_ANG) = {'deg','deg','deg'};
p.X_scale(p.X_ANG) = [1,1,1]*r2d;

p.X_names(p.X_VEL) = {'xdot','ydot','zdot'};
p.X_units(p.X_VEL) = {'m/s','m/s','m/s'};
p.X_scale(p.X_VEL) = [1,1,1];

p.X_names(p.X_POS) = {'x','y','z'};
p.X_units(p.X_POS) = {'m','m','m'};
p.X_scale(p.X_POS) = [1,1,1];

p.X_names(p.X_BA) = {'ba-1','ba-2','ba-3'};
p.X_units(p.X_BA) = {'mg','mg','mg'};
p.X_scale(p.X_BA) = [1,1,1]*1000/p.grav;

p.X_names(p.X_BG) = {'bg-1','bg-2','bg-3'};
p.X_units(p.X_BG) = {'deg/hr','deg/hr','deg/hr'};
p.X_scale(p.X_BG) = [1,1,1]*r2d*3600;

% input indices
p.U_INPUTS = 12;
p.U_NA = 1:3;
p.U_NBA = 4:6;
p.U_NG = 7:9;
p.U_NBG = 10:12;

% location parameters
p.g_ib_t = [0; 0; 9.78];
p.omega_it_t = [0; 0; 0];
p.declination = 13*d2r;

% noise parameters (we should measure these quantities!)
p.sigma_na = 1e-4;
p.sigma_nba = 1e-6;
p.sigma_ng = 0.00011636;
p.sigma_nbg = 1e-6;
% 
% p.sigma_na = 1e-4;
% p.sigma_nba = 5e-4;
% p.sigma_ng = 1e-3;
% p.sigma_nbg = 5e-4;

% p.sigma_na = 0.25e-6;
% p.sigma_nba = 5.0e-9;
% p.sigma_ng = 1.9e-5;
% p.sigma_nbg = 5.0e-12;
% 
% p.sigma_na = 5.0e-4;
% p.sigma_nba = 5.0e-6;
% p.sigma_ng = 5.0e-8;
% p.sigma_nbg = 5.0e-8;

% identify correction types
p = add_correction(p, 'GPS', 'm', 1, 'b', 1);



function p = add_correction(p, name, unit, scale, color, count)
if ~isfield(p, 'M_names')
	p.M_names = {};
	p.M_colors = {};
	p.M_scale = [];
	p.M_units = {};
end
n = length(p.M_names);
p = setfield(p, sprintf('m_%s', name), (n+1):(n + count));
for i = 1:count
	idx = n + i;
	if count > 1
		p.M_names{idx} = sprintf('%s-%d', name, i);
	else
		p.M_names{idx} = name;
	end
	p.M_units{idx} = unit;
	p.M_scale(idx) = scale;
	p.M_colors{idx} = color;
end
