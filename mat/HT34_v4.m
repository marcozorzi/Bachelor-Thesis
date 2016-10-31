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
L_mot = 1.7;                                            % Inductance [H]


time = [0:t_comm:time_total];
[uno, samples] = size(time);

%desired movement law plot.
ii = 1;
desired_law = zeros(1,samples);
desired_law(ii) = 0;
m1 = (speed_peak)/(floor(samples*0.2));
q1 = speed_peak - (floor(samples*0.8)+1)*(-m1);
for ii = 1:(floor(samples*0.2))
    desired_law(ii)= ii*m1;
end
for ii = (floor(samples*0.2)+1):(floor(samples*0.8))
    desired_law(ii) = speed_peak;
end
for ii = ((floor(samples*0.8))+1):(samples)
    desired_law(ii)= -ii*m1+q1;
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
desired_law_motor = zeros(1,samples);
desired_law_motor(ii) = 0;
for ii = 1:(samples)
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
torque = zeros(1,samples);
for ii = 2:(samples)
   torque(ii)= Jeq*((desired_law_motor(ii)-desired_law_motor(ii-1))/t_comm) + TL;
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
desired_law_rpm = zeros(1,samples);
desired_law_rpm(ii) = 0;
for ii = 1:(samples)
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
interval = 2*pi;
th_delta = (interval/samples);
th(1) = 0;
current1 = zeros(1, samples);
ii = 1;
for ii = 1:samples
    th(ii+1) = th(ii) + th_delta;
    temp1 = (-0.5*torque(ii))/(L_mot*sin(4*th(ii)));
    current1(ii) = sqrt(abs(temp1));
    current_cost(ii) = sqrt((torque(ii))/(2*L_mot));
    %{
    temp2 = (-0.5*torque(ii))/(L_mot));
    current2(ii) = sqrt(abs(temp2));
    temp3 = (-0.5*torque(ii))/(L_mot*sin(4*th(ii)-(4/3)*pi));
    current3(ii) = sqrt(abs(temp3));
    %}
end


figure(6);
plot(time, current1);
axis([0 time_total 0 1]);
hold on;
plot(time, current_cost, 'red');
grid minor;


%{
plot(time, current2, 'red');
plot(time, current3, 'green');



for ii = 1:samples
    coppia1(ii) = -2*L_mot*4.5*sin(4*th(ii));
    coppia2(ii) = -2*L_mot*4.5*sin(4*th(ii)-(2/3)*pi);
    coppia3(ii) = -2*L_mot*4.5*sin(4*th(ii)-(4/3)*pi);
    coppiamia(ii) = torque(ii);
end



figure(7);
plot(time, coppia1);
grid minor;
hold;
plot(time, coppia2, 'red');
plot(time, coppia3, 'green');
plot(time, coppiamia);

%}



display('---------------------------HT34_v1 run succesfully---------------------------')