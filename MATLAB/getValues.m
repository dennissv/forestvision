% Use this file to get values from INS
clear all
filename = 'snovag-copy.txt';

[position_B, sd_B, position, vel_NEU, angles, GPStime, time, ...
    pos_xyz, sd_xyz, vel_xyz, sd_velxyz, pos_cov, att_cov, vel_cov] ...
    = readins(filename);

% Output: position_B - latitude, longitude and height from BESTPOS
%         sd_B - standard deviation for lat, long and height (BESTPOS)
%         position - latitude, longitude and height from INSPVA
%         vel_NEU - velocities North, East and Up from INSPVA
%         angles - roll, pitch, yaw from INSPVA
%         GPStime - GPSweek and seconds from week start from INSPVA
%         time - year, month, day, hourUTC, minute, milliseconds from TIME
%         pos_xyz - x, y, z position from BESTXYZ
%         sd_xyz - standard deviation for x, y, z from BESTXYZ
%         vel_xyz - x, y, z velocities from BESTXYZ
%         sd_velxyz - standard dev. for x, y, z velocities from BESTXYZ
%         pos_cov - position covariance from INSCOV
%         att_cov - attitude covariance from INSCOV
%         vel_cov - velocity covariance from INSCOV

[x,y,utmzone] = deg2utm(position(:,1),position(:,2));
x = x - x(1);
y = y-y(1);
plot(x,y);