%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function reads the given file with data from the INS 
% and saves them in corresponding variables 
%
% Input: filename
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [position_B, sd_B, position, vel_NEU, angles, GPStime, time, ...
    pos_xyz, sd_xyz, vel_xyz, sd_velxyz, pos_cov, att_cov, vel_cov] ...
    = readins(filename)

% Open file and read first line of data
fid=fopen(filename);
tline=fgets(fid); % First line
i=1; % Line counter
n=0; % Number of lines that can't be read

% Counters for different type of logs
c1=0;
c2=0;
c3=0;
c4=0;
c5=0;

% Read all lines and save to AllData
while ischar(tline)
    AllData{i}=tline;
    i=i+1;
    tline=fgets(fid);
end
fclose(fid); % Close file

s=size(AllData);

% Loop over all data and add to corresponding log type
for i=1:s(2)
    C=strsplit(AllData{i},';'); % Split header
    str=strjoin(C);             % Convert to string
    C2=strsplit(str,',');       % Split values into different cells 
    
    % Check which type of log
    % Save data to correct log type
    s2=size(C2);
    if strcmp(C2{1,1},'#BESTPOSA')==1
        c1=c1+1;
        BESTPOS(c1,1:s2(2))=C2;
    elseif strcmp(C2{1,1},'#TIMEA')==1
        c2=c2+1;
        TIME(c2,1:s2(2))=C2;
    elseif strcmp(C2{1,1},'#INSCOVA')==1
        c3=c3+1;
        INSCOV(c3,1:s2(2))=C2;
        x = strsplit(INSCOV{c3,38},'*');
        INSCOV(c3,38) = x(1);
    elseif strcmp(C2{1,1},'#INSPVAA')==1
        c4=c4+1;
        INSPVA(c4,1:s2(2))=C2;
    elseif strcmp(C2{1,1},'#BESTXYZA')==1
        c5=c5+1;
        BESTXYZ(c5,1:s2(2))=C2;
    else 
        n=n+1;
    end
    
end

% Receive different data from different log types 

if exist('BESTPOS','var')
    % Position data
    latitude_B = str2double(BESTPOS(:,12));
    longitude_B = str2double(BESTPOS(:,13));
    height_B = str2double(BESTPOS(:,14));
    position_B = [latitude_B longitude_B height_B];

    % Velocities
    sd_latB = str2double(BESTPOS(:,17));
    sd_longB = str2double(BESTPOS(:,18));
    sd_hB = str2double(BESTPOS(:,19));
    sd_B = [sd_latB sd_longB sd_hB];
else
    position_B = [];
    sd_B = [];
end

if exist('INSPVA','var')
    % Position data
    latitude = str2double(INSPVA(:,12));
    longitude = str2double(INSPVA(:,13));
    height = str2double(INSPVA(:,14));
    position = [latitude longitude height];

    % Velocities
    vel_north = str2double(INSPVA(:,15));
    vel_east = str2double(INSPVA(:,16));
    vel_up = str2double(INSPVA(:,17));
    vel_NEU = [vel_north vel_east vel_up];

    % Angles
    roll = str2double(INSPVA(:,18));
    pitch = str2double(INSPVA(:,19));
    yaw = str2double(INSPVA(:,20));
    angles = [roll pitch yaw];
    
    % GPS week and seconds into week
    GPSweek = str2double(INSPVA(:,6));
    second_GPS = str2double(INSPVA(:,7));
    GPStime = [GPSweek second_GPS];
else
    position = [];
    vel_NEU = [];
    angles = [];
    GPStime = [];
end

if exist('TIME','var')
    % Date and time
    year = str2double(TIME(:,14));
    month = str2double(TIME(:,15));
    day = str2double(TIME(:,16));
    hourUTC = str2double(TIME(:,17));
    minute = str2double(TIME(:,18));
    millisecond=str2double(TIME(:,19));
    time = [year month day hourUTC minute millisecond];
else
    time = [];
end


if exist('BESTXYZ','var')
    % Best positions, xyz (cartesian coordinates)
    pos_x = str2double(BESTXYZ(:,12));
    pos_y = str2double(BESTXYZ(:,13));
    pos_z = str2double(BESTXYZ(:,14));
    pos_xyz = [pos_x pos_y pos_z];

    % Standard deviation positions, xyz (cartesian coordinates)
    sd_x = str2double(BESTXYZ(:,15));
    sd_y = str2double(BESTXYZ(:,16));
    sd_z = str2double(BESTXYZ(:,17));
    sd_xyz = [sd_x sd_y sd_z];
    
    % Velocities, xyz (cartesian coordinates)
    vel_x = str2double(BESTXYZ(:,20));
    vel_y = str2double(BESTXYZ(:,21));
    vel_z = str2double(BESTXYZ(:,22));
    vel_xyz = [vel_x vel_y vel_z];

    % Standard deviation velocities, xyz (cartesian coordinates)
    sd_velx = str2double(BESTXYZ(:,23));
    sd_vely = str2double(BESTXYZ(:,24));
    sd_velz = str2double(BESTXYZ(:,25));
    sd_velxyz = [sd_velx sd_vely sd_velz];
else
    pos_xyz = [];
    sd_xyz = [];
    vel_xyz = [];
    sd_velxyz = [];
end

if exist('INSCOV','var')
    % Position covariance
    pos_cov = str2double(INSCOV(:,12:20));
    % Attitude covariance
    att_cov = str2double(INSCOV(:,21:29));
    % Velocity covariance
    %velcov9 = strsplit(INSCOV(:,38),'*');
    %INSCOV(:,38) = velcov9(:,1);
    vel_cov = str2double(INSCOV(:,30:38));
else 
    pos_cov = [];
    att_cov = [];
    vel_cov = [];
end

