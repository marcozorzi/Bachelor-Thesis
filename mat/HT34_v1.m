%Torque calculation and HT34-504 simulator

%enviroment preparation
close all;
clear all;
clc

%motor parameters
Jm = 1.1*10^-4;                                         % motor inertia [kg*m^2]
alfa_p = 1.8;                                           % step angle [degrees]
Np = 360/alfa_p;                                        % steps number for one revolution

%ball screw parameters
Jv = 2.10*10^-4;                                        % screw inertia [kg*m^2]
lead = 0.025;                                           % righ hand lead [m]
r_eq = lead/(2*pi);                                     % equivalent radius [m]
diam = 0.025;                                           % screw diameter [m]. Not used
efficiency = 0.85;

%System requirements.
F = 270;                                                % Force necessary to move the load. [N]
time_total = 0.25;                                      % total time to move the load. [s]
time_rise = 0.2*time_total;                             % time in which the load accelerates. [s]
path_l = 30;                                            % Total linear excursion of the lever [mm]
speed_medium = (path_l*10^-3) / time_total;             % linear and medium speed of the load.
speed_peak= (speed_medium*time_total)/(time_total-time_rise);   % top linear speed. [m/s]
radial_speed_peak = speed_peak/r_eq;                    %top radial speed reached by the motor [m/s]
freq_init = 800;                                        % initial step rate [sps]
freq_fin = (radial_speed_peak/pi)*Np;                   % final step rate [sps]
TL = (F*speed_peak)/(radial_speed_peak*efficiency);     % Load Torque


time = [0:0.001:time_total];

%desired movement law plot.
ii = 2;
desired_law = zeros(1,time_total*100+1);
desired_law(ii) = 0;
for ii = 2:(time_total*200)
    desired_law(ii)= desired_law(ii-1) + 0.003;
end
for ii = (time_total*200+1):(time_total*800)
    desired_law(ii) = 0.15;
end
for ii = (time_total*800+1):(time_total*1000+1)
    desired_law(ii)= desired_law(ii-1) - 0.003;
end
figure(1);
plot(time,desired_law)
axis([0 time_total 0 speed_peak+0.1*speed_peak])
grid
title('Desired Movement Law. Linear speed / time')
xlabel('Time (s)');
ylabel('Speed (m/s)');

%desired movement law for the motor.
ii = 1;
desired_law_motor = zeros(1,time_total*1000+1);
desired_law_motor(ii) = 0;
for ii = 1:(time_total*1000)
    desired_law_motor(ii)= desired_law(ii)*((2*pi)/lead);
end

figure(2);
plot(time,desired_law_motor)
axis([0 time_total 0 radial_speed_peak+0.1*radial_speed_peak])
grid
title('Desired Movement Law. radial speed / time')
xlabel('Time (s)');
ylabel('Speed (rad/s)');


%torque calculation
Jeq = (Jm+Jv/efficiency);
ii = 1;
torque = zeros(1,time_total*1000+1);
for ii = 2:(time_total*1000)
   torque(ii)= Jeq*((desired_law_motor(ii)-desired_law_motor(ii-1))/0.001) + TL;
end

figure(3);
plot(time,torque);
axis([0 time_total 0 max(torque)*1.05])
grid minor
title('Torque required / Time')
xlabel('Time (s)');
ylabel('Torque (Nm)');


%torque / speed figure
ii = 1;
desired_law_rpm = zeros(1,time_total*1000+1);
desired_law_rpm(ii) = 0;
for ii = 1:(time_total*1000+1)
    desired_law_rpm(ii)= (desired_law_motor(ii)*60)/(2*pi);
end
figure(4);
plot(desired_law_rpm,torque);
axis([0 max(desired_law_rpm)*1.05 0 max(torque)*1.05])
grid 
title('Torque required / Speed')
xlabel('Speed (rpm)');
ylabel('Torque (Nm)');

clear ii
pullout_speed_or = [0:150:1800];
pullout_speed = [0:150:1800];
pullout_torque_or = zeros(1, 13);
pullout_torque_or = [2.69 2.69 2.69 1.91 1.27 1.06 0.96 0.85 0.74 0.64 0.53 0.42 0.32];

pullout_speed(3)
pullout_torque_or(3)
for aa = 1:11
    a = float('double');
    
    b = pullout_torque_or(jj);
    diff = a - b;
    m = (diff) / 150;
    for ii = 1:150
        pullout_torque(ii*150+jj) = pullout_torque_or(jj+1) + m*(pullout_speed(ii+1)-pullout_speed(ii)); 
    end
end




display('---------------------------HT34_v1 run succesfully---------------------------')