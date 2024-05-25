% Take Home Exam - Transfer Function 9 
% Name: Kobe Michiels (r0848543)

% Question 1 (System A):
clear
% Measure and visualize the step response and bode plot of your unknown
% system A.

%Step response:
time_in = (0:1e-3:1)';
input = heaviside(time_in);
simOut = sim('transferfunction9A_harness.slx',time_in);
figure('Name', 'Generated step response for unknown system A');
plot(time_in,simOut.response)
title('Step response for unknown system A')

%% Bode plot:
clear

% Initialisation of time vector
startTime = 0;
endTime = 50;
samplingTime = 0.0001;
time_in = (startTime:samplingTime:endTime)';

% Initialisation of frequency (in rad/s)
startFreq = 1;  
endFreq = 1000;   
numPoints = 100; 
freq = logspace(log10(startFreq), log10(endFreq), numPoints);

% Initialisation of magnitude and phase vector to store generated data:
magnitude = zeros(1, numPoints);
phase = zeros(1, numPoints); 

% Generation of Bode Plot data
for k = 1 : numPoints
    input = sin(freq(k)*time_in);
    simOut = sim("transferfunction9A_harness",time_in);
    output = simOut.response;

    % Magnitude
    magnitude(k) = max(abs(output)) / max(abs(input));

    % Phase
    % Local maximum of input after possible transition phenomena disappear:
    for i = (0.75*(length(time_in)-1)):length(time_in)
        if input(i-1) < input(i) && input(i+1) < input(i)
            input_index = i;
            input_time = input_index*samplingTime;
            break;
        end
    end
    
    % First next local maximum of output:
    for i = input_index:length(time_in)
        if output(i-1) < output(i) && output(i+1) < output(i)
            output_index = i;
            output_time = output_index*samplingTime;
            break;
        end
    end

    % Calculate phase with time delay:
    phase(k) = -(output_time-input_time)*freq(k)*180/pi; 
    if phase(k) < -350
        phase(k) = phase(k) + 360;
    end
end
% Save data to MATLAB Data file:
save("transferfunction9A_BodePlotWorkspace.mat")

%% Generate Bode plot with MATLAB Data file data:
load("transferfunction9A_BodePlotWorkspace.mat")
figure('Name', 'Generated bode plot for unknown system A');
subplot(2,1,1);
semilogx(freq, 20*log10(magnitude));
title('Bode Plot for unknown system A')
ylabel('Magnitude (dB)');
xlim([1 1000]);
ylim([-30 5]);
yticks(-30:5:5)
grid on;
subplot(2,1,2);
semilogx(freq, phase);
xlabel('Frequency (rad/s)');
ylabel('Phase (degrees)');
xlim([1 1000]);
ylim([-135 45]);
yticks(-135:45:45)
grid on;

%% Question 2 (System A): 
clear
% Reverse engineer the transfer function from your Bode Plot and/or step
% response.

%Reverse engineered transfer function: 
num_A = [35, 2304];
den_A = [1, 46, 3600];
TF_A = tf(num_A, den_A);

% Bode plot of reverse engineered transfer function: 
figure('Name', 'Reversed engineered bode plot for unknown system A');
bode(TF_A)
title('Reversed engineered bode plot for unknown system A')
xlim([1 1000]);
grid on

% Step response of reverse engineered transfer function:
figure('Name', 'Reversed engineered step response for unknown system A');
step(TF_A)
title('Reversed engineered step response for unknown system A')
xlim([0 1])

%% Question 3 (System B):
clear
% Discuss the (in)stability of the system. Make a root locus plot and find
% gain and phase margin.

num_B = [3114, 45598];
den_B = [1, -22, 19321];
TF_B= tf(num_B, den_B);

% Step response:
figure('Name', 'Step response for system B');
step(TF_B)
title('Step response for system B')
% Pole-zero-map:
figure('Name', 'Pole-zero-map for system B');
pzmap(TF_B)
title('Pole-zero-map for system B')
% Root locus plot:
figure('Name', 'Root locus plot of system B');
rlocus(TF_B)
title('Root locus plot of system B')
%Gain and phase margin:
figure('Name', 'Gain and phase margin of system B');
margin(TF_B)

%% Build a P controller around the unprotected transfer function (B) to 
% stabilize the system. Tune the P controller so the system is critically 
% damped.

Kc = 0.0867;
t = (0:1e-3:1)';
input = heaviside(t);
out = sim("transferfunction9B.slx",t);
figure('Name', 'Step response for system B - Critically damped');
plot(t,out.y) 
title('Step response for system B - Critically damped')

%% Question 4 (System A):
clear
% Build a control loop around the protected transfer function. 
% Tune the PI(D) controller to have less than 10% overshoot. 
% The D-action may or may not be necessary

time_in = (0:1e-3:1)';
input = heaviside(time_in);
Kc = 1.5;
Ti = 0.01;
Td = 0.003;
tau_d = Td/(10*Kc);
time_delay = 1e-12;
simOut = sim('transferfunction9A_harness_with_control_loop.slx',time_in);
figure('Name', 'Step response for system A with PID - 10% overshoot');
plot(time_in,simOut.response)
title('Step response for system A with PID - 10% overshoot')

%% Question 5 (System A): 
clear
%  In Simulink, add a time delay in series with system A. 
% Tune the PI(D) controller again so the output is desirable even with this
% delay. 

time_in = (0:1e-3:1)';
input = heaviside(time_in);
Kc = 0.25;
Ti = 0.01;
Td = 0.01;
tau_d = Td/(10*Kc);
time_delay = 0.02;
simOut = sim('transferfunction9A_harness_with_control_loop.slx',time_in);
figure('Name', 'Step response for system A with PID and time delay');
plot(time_in,simOut.response)
title('Step response for system A with PID and time delay')

% Calculate or find the maximum delay for which the system can be made
% stable.

[Gm,Pm,Wcg,Wcp] = margin(tf([35, 2304], [1, 46, 3600]));
max_time_delay = Pm*pi/(180*Wcp)
