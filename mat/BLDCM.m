%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Brushless DC Motor Simulator %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% INITIAL VARIABLE DECLARATION

time = [0:0.1:300];
Kt = 14.2*10^-3; %Nm/A

%% MOTOR PARAMETERS
% Maxon motor 60 W
Jm = 2.19*10^-6;         % motor inertia [kg*m^2]

%% GEARBOX PARAMETERS
Jv = 7*10^-8;            % screw inertia [kg*m^2]
efficiency = 0.624;      % 

%% SYSTEM REQUIREMENTS
F = 270;                                                        % Force necessary to move the load. [N]
time_total = 0.25;                                              % total time to move the load. [s]
time_rise = 0.2*time_total;                                     % Time in which the load accelerates. [s]
movement_angle = 34;                                            % degrees of engine clutch lever movement
movement_angle_rad = (movement_angle*pi)/180;                   
mean_angular_speed = movement_angle_rad / time_total;
speed_medium = (mean_angular_speed * 0.037) + 0.0002;           % linear and medium speed of the load compensated for truncation.
speed_peak= (speed_medium*time_total)/(time_total-time_rise);   % top linear speed. [m/s]
radial_speed_peak = speed_peak/r_eq ;                           %top radial speed reached by the motor [m/s]
TL = (F*speed_peak)/(radial_speed_peak*efficiency);             % Load Torque


time = [0:0.001:time_total];                                    %Time vector
[uno, samples] = size(time);                                    %Number of time units

%% DESIRED MOTION LAW PLOT
% In this section the plot for the desired motion is implemented
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

fig1 = figure(1);
plot(time,desired_law)
axis([0 time_total 0 speed_peak+0.1*speed_peak])
grid
hold on
title('Desired Motion Law. Linear speed / time')
xlabel('Time (s)');
ylabel('Speed (m/s)');
vfissa = ones(size(time))*speed_medium;
plot(time, vfissa, '--r');
legend('Desired Motion Law','Average speed = 0.088 m/s');
saveas(fig1, 'legge_moto2.pdf');


%% DESIRED MOTION LAW PLOT FOR THE MOTOR
% The desired motion law is transferred to the motor through the ball screw
ii = 1;
desired_law_motor = zeros(1,samples);
desired_law_motor(ii) = 0;
for ii = 1:(samples)
    desired_law_motor(ii)= desired_law(ii)*((2*pi)/lead);
end

fig2 = figure(2);
plot(time,desired_law_motor)
axis([0 time_total 0 radial_speed_peak+0.1*radial_speed_peak])
grid
title('Desired speed Law. for the motor - Angular speed vs time')
xlabel('Time (s)');
ylabel('Speed (rad/s)');
saveas(fig2, 'legge_moto_motore.pdf');