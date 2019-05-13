clc; clear;  

%{
Author          : Tumisang Kalagobe

Date finalised  : 16/04/2019

Description     : File used to complete the MECN4029 wood cutting machine
project for 2019, the project brief for which can be found in "Brief.pdf".

The file contains analyses for the performance for a Qingdao
Saifaan wood cutting machine in both the time and frequency domain. The 
file is divided into a number of key parts. 

The first is an initialisation
of the cutter model parameters such as the motor characteristics, shaft
stiffnesses, cutting blade parameters, etc.

The second calls the Simulink model used to analyse the performance of the
controlled and uncontrolled model.

A full description of the model can be found in the
"MECN4029_800363.pdf" file uner the "Modelling" section.
%}

%%%%%%%%%%%%%%%%%%%%%% Simulation parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_stop  = 30;           % duration of the simulation (s)

% Initialising base parameters for the cutter, motor and shaft

N_m = 1500;             % motor angular velocity (rpm)
V_c = 100;               % cutting blade diameter (m)
D_c = 0.45;             % blade diameter (m)
w_c = 2*V_c/D_c;        % desired cutting blade angular velocity (rad/s)
w_m = (pi/30)*N_m;      % motor angular velocity (rad/s)
w_c_d = 0;
w_c_dd = 0;
P_cut = 3.8063e3;       % cutting power required (W)

%%%% motor characteristics %%%%
e_a = 97;                  % motor input voltage (V)
R_a = 0.65;                 % motor armature resistance (Ohm)
L_a = 6.6e-3;               % motor armature inductance (Henries)
P_m = 23.2e3;               % motor input power (W)
i_a = 7.5;                  % rated armature current (A)
T_rated = 18;               % rated motor torque (N.m) 
K_t = T_rated/i_a;          % motor torque constant (N.m/A)
K_e = 0.149;
J_m = 0.0064;               % motor moment of inertia (kg.m^2)
b_motor = 5e-3;             % motor damping coefficient (N.s/m)

%%% Cutting Process characeristics %%%
rho_c = 8000;               % cutting blade density (kg/m^3)
t = 2.89e-3;                % cutting blade thickness (m)
J_c = (pi*rho_c*t*D_c^4)/(32); % blade mass moment of intertia (kg.m^2)
b_1 = 5e2;                  % motor side bearing damping constant (N.s/m)
b_2 = 5e2;                  % cutting side damping constant (N.s/m)
c = 2*P_cut/(w_c^3);        % load torque constant (N.m.s^2)
l_shaft = .5;               % shaft length (m)
G_steel = 73.1e9;           % steel shear modulus (Pa)
d_shaft = 0.03;             % shaft diameter (m)
k = (G_steel*pi*d_shaft^4)/(32*l_shaft);   % shaft stiffness constant (N/m)

%%% Measurement system characteristics %%%
K_tach = 0.0289;            % tachometer gain

%%%%%%%%%%%%%%%%%%%%%%% Frequency domain analysis %%%%%%%%%%%%%%%%%%%%%%%%%

%%% Motor frequency domain characteristics %%%
K = K_t/(L_a*J_m);                          % constant gain
c2 = 1;                                     % coeffficient of s^2 term
c1 = (L_a*b_motor + R_a*J_m)/(L_a*J_m);     % coeficient of s^1 term
c0 = (R_a*b_motor + K_e^2)/(L_a*J_m);       % coeficient of s^0 term

num_Motor = [J_m b_motor];                  % numerator for motor tsf fn
den_Motor = [c2 c1 c0];                     % denominator for motor tsf fn
transferFunction_Motor = K*tf(num_Motor, den_Motor);% motor tsf fn

%%% Plant frequency domain characteristics %%%
a0 = (2*c*J_m*w_c_dd + 2*k*c*w_c)/(J_m*J_c);% coeficient of s^0 term
a1 = ((1/(J_m*J_c))*(J_m*b_2 + J_c*b_1 + J_m*k + b_2*b_1 + J_c*k... 
    + k*(b_1+b_2) + 4*c*J_m*w_c_d));  % coeficient of s^1 term
a2 = (1/(J_m*J_c))*2*c*J_m*w_c;        % coeffficient of s^2 term
a3 = 1;                     % coefficient of s^3 term

num_Plant = [k/(J_m*J_c)];  % numerator of the plant transfer function
den_Plant = [a3 a2 a1 a0];  % denominator of the plant transfer function
transferFunction_Plant = tf(num_Plant, den_Plant);  % plant transfer fn

transferFunction = transferFunction_Motor*transferFunction_Plant; % full...
% ...uncontrolled system transfer function

A = [0  1   0
    0   0   1 
    a0  a1  a2]; % plant linear model matrix

eigenValues = eig(A);

%%% Bode plot for the open-loop uncontrolled system %%%
% figure('Visible', 'off')
% bode(transferFunction)
% grid on
% print(gcf,'Plots/bodePlot','-dpng','-r500'); 

bandwidth = bandwidth(transferFunction);    % uncontrolled system bandwidth

%%% Pole-Zero plot of the open-loop uncontrolled system %%%
% figure('Visible', 'off')
% pzmap(transferFunction);
% print(gcf,'Plots/poleZeroPlot','-dpng','-r500');   

[poles, zeros] = pzmap(transferFunction);   % uncontrolled system P's & Z's

%%% Nyquist plot of the open loop uncontrolled system %%%
% figure('Visible', 'off')
% nyquist(transferFunction);
% print(gcf,'Plots/nyquistPlot','-dpng','-r500'); 

%%%%%%%%%%%%%%%%%%%%%%%%% Controller design %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_cut_start = 5;% time when cutting engages
t_cut_stop = 1.5*t_cut_start; % time when cutting disengages
t_step = 0;
%%% PID Controller %%%
p_PID    = 1.6018878881e11;   % proportional gain
i_PID    = 0; % integral gain 
d_PID    = 0;   % derivative gain
N = 100;

num_pidController = p_PID*[1+N*d_PID N i_PID];
den_pidController = [1 N 0];
pidControllerTF = tf(num_pidController, den_pidController);

pidSystemTF = (transferFunction*pidControllerTF)/(1 + transferFunction*...
    pidControllerTF*K_tach);

step(pidSystemTF)
%figure(2); bode(pidSystemTF);
%[poles_pid, zeros_pid] = pzmap(transferFunction);   % uncontrolled system P's & Z's

% if real(poles_pid) > 0
%     disp("PID Controlled system is unstable!")
% else 
%     disp("PID Controlled system is stable :-)")
% end 

%%% PDF Controller gains %%%
p_PDF    = 0;   % proportional gain
i_PDF    = 0;   % integral gain
d_PDF    = 0;   % derivative gain

%%%%%%%%%%%%%%%%%%%%%%%%% SImulink execution %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Running the simulink model %%% 
simulinkFile = "automaticWoodCuttingMachine.slx";
%sim(simulinkFile);

%%% Open Loop Model %%%
openLoopSim = "openLoop.slx";
%sim(openLoopSim);

%%% PID Model %%%
pidSimulink = "pidControlled.slx";
%sim(pidSimulink);

%%% PDF Model %%%
pdfSimulink = "pdfControlled.slx";
%sim(pdfSimulink);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% END OF FILE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%