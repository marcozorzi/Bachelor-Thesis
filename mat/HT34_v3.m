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
t_comm = (1/freq_fin)*10^5;
t_comm = round(t_comm);
t_comm = t_comm *10^-5;
TL = (F*speed_peak)/(radial_speed_peak*efficiency);     % Load Torque


time = [0:t_comm:time_total];
[uno, samples] = size(time);

%desired movement law plot.
ii = 2;
desired_law = zeros(1,samples);
desired_law(ii) = 0;
for ii = 2:(floor(samples*0.2))
    desired_law(ii)= desired_law(ii-1) + 0.003;
end
for ii = (floor(samples*0.2)+1):(floor(samples*0.8))
    desired_law(ii) = 0.15;
end
for ii = ((floor(samples*0.8))+1):(samples)
    desired_law(ii)= desired_law(ii-1) - 0.003;
end
figure(1);
plot(time,desired_law)
%axis([0 time_total 0 speed_peak+0.1*speed_peak])
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


pullout_speed_or = [0:150:1800];
pullout_speed = [0:1:1800];
pullout_torque_or = zeros(1, 13);
pullout_torque_or = [2.69 2.69 2.69 1.91 1.27 1.06 0.96 0.85 0.74 0.64 0.53 0.42 0.32];

for jj = 1:12
    m = (pullout_torque_or(jj+1) - pullout_torque_or(jj)) / 150;
    q = pullout_torque_or(jj) - m * pullout_speed_or(jj);
    for ii = 1:150
        index = (jj-1)*150+ii;
        if index < 1801
            pullout_torque(index) =  m*pullout_speed(index) + q; 
        end
    end
end

pullout_torque(1801) = 0;
pullout_torque_smooth = smooth(pullout_torque,500);

[rows, columns] = size(desired_law_rpm);

for ii = columns:1801
   desired_law_rpm(ii) = 0;
   torque(ii) = 0;
end


figure(5);
plot(pullout_speed, pullout_torque_smooth);
hold;
plot(desired_law_rpm, torque, 'red');
grid;
axis([0 max(pullout_speed)*1.05 0 max(pullout_torque_smooth)*1.05]);
title('Torque required and pull-out curve/ Speed')
xlabel('Speed (rpm)');
ylabel('Torque (Nm)');
legend('Pull-out curve', 'required torque');


%coppia T = -2*L*i^2*sin(4th)


display('---------------------------HT34_v1 run succesfully---------------------------')