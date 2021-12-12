%%--------------------------------------------------------------------
% AUTHOR: Joyce Lai
% DATE: 2021 December
%
% DESCRIPTION
%
% This script tests the performance of a water-reservoir system with and
% without different controllers. We test the system with no controller, 2
% P controllers, 1 PI controller, 1 PD controller, and 2 PID controllers.
% Generally, the system's forward path consists of the PID controller,
% C(s), motor, P(s), and STH block, G(s), all in series. The system has
% unity feedback.
%
% This script was originally written for an automatic control systems
% project, but has been modified slightly.
%%--------------------------------------------------------------------
%% CLEAR ALL, CLOSE ALL
    clc
    clear all
    close all
    
%% DEFINE VARIABLES
    verify_tf = true;           % Verify MATLAB computed and analytical transfer functions match
    verify_plot_step = true;   % Plot step responses of MATLAB computed and analytical transfer functions together
    step_info = [];             % Declare empty array for all step info (do not change this!)
    
%% NO CONTROLLER
% Calculate closed-loop transfer function without PID controller
    [T1,T1_fwd] = water_reservoir.compute_tf(0,0,0,false);
    
    if verify_tf
        T1_analytical = water_reservoir.compute_tf_analytical(0,0,0,false);
        T1_matches = water_reservoir.tf_matches(T1,T1_analytical,verify_plot_step)
    end

%% P CONTROLLER
% Calculate closed-loop transfer function without PID controller, Kp=5
    [T2,T2_fwd] = water_reservoir.compute_tf(5,0,0,true);
    
    if verify_tf
        T2_analytical = water_reservoir.compute_tf_analytical(5,0,0,true);
        T2_matches = water_reservoir.tf_matches(T2,T2_analytical,verify_plot_step)
    end

% Calculate closed-loop transfer function without PID controller, Kp=10
    [T3,T3_fwd] = water_reservoir.compute_tf(10,0,0,true);
    
    if verify_tf
        T3_analytical = water_reservoir.compute_tf_analytical(10,0,0,true);
        T3_matches = water_reservoir.tf_matches(T3,T3_analytical,verify_plot_step)
    end

%% NO CONTROLLER AND P CONTROLLER - STEP
% Step responses
    figure(1);
    hold on
    step(T1);
    step(T2);
    step(T3);
    grid on
    hold off
    legend('No PID','Kp = 5','Kp = 10');
% Extract step info
    T1_stepinfo = water_reservoir.extract_stepinfo_data(T1);
    T2_stepinfo = water_reservoir.extract_stepinfo_data(T2);
    T3_stepinfo = water_reservoir.extract_stepinfo_data(T3);
    step_info = [step_info; T1_stepinfo; T2_stepinfo; T3_stepinfo];

%% NO CONTROLLER AND P CONTROLLER - BODE, ROOT LOCUS
    close all
% Bode plots
    figure;
    hold on
    bode(T1_fwd); bode(T2_fwd); bode(T3_fwd);
    grid on
    hold off
    legend('No PID','Kp = 5','Kp = 10')
% Root locus (using closed-loop)
    figure;
    hold on
    rlocus(T1_fwd); rlocus(T2_fwd); rlocus(T3_fwd);
    grid on
    hold off
    legend('No PID','Kp = 5','Kp = 10');

%% PI CONTROLLER
% Calculate closed-loop transfer function with PI controller, Kp=10, Ki=10
    [T4,T4_fwd] = water_reservoir.compute_tf(10,10,0,true);
    
    if verify_tf
        T4_analytical = water_reservoir.compute_tf_analytical(10,10,0,true);
        T4_matches = water_reservoir.tf_matches(T4,T4_analytical,verify_plot_step)
    end
        
%% PI CONTROLLER - STEP
% Step response
    figure;
    step(T4);
    grid on
    legend('Kp=Ki=10');
% Extract step info
    T4_stepinfo = water_reservoir.extract_stepinfo_data(T4);
    step_info = [step_info; T4_stepinfo];
    
%% PI CONTROLLER - BODE, ROOT LOCUS
% Bode plot
    figure;
    bode(T4_fwd);
    grid on
    legend('Kp=Ki=10');
% Root locus (using closed-loop)
    figure;
    rlocus(T4_fwd);
    grid on
    legend('Kp=Ki=10');
    
%% PD CONTROLLER
% Calculate closed-loop transfer function with PD controller, Kp=10, Kd=5
    [T5,T5_fwd] = water_reservoir.compute_tf(10,0,5,true);
    
    if verify_tf
        T5_analytical = water_reservoir.compute_tf_analytical(10,0,5,true);
        T5_matches = water_reservoir.tf_matches(T5,T5_analytical,verify_plot_step)
    end

%% PD CONTROLLER - STEP
% Step response
    figure;
    step(T5);
    grid on
    legend('Kp=10, Kd=5');
% Extract step info
    T5_stepinfo = water_reservoir.extract_stepinfo_data(T5);
    step_info = [step_info; T5_stepinfo];

%% PD CONTROLLER - BODE, ROOT LOCUS
% Bode plot
    figure;
    bode(T5_fwd);
    grid on
    legend('Kp=10, Kd=5');
% Root locus (using closed-loop)
    figure;
    rlocus(T5_fwd);
    grid on
    legend('Kp=10, Kd=5');

%% PID 1 CONTROLLER
% Calculate closed-loop transfer function with PID controller, Kp=12, Ki=15, Kd=3
    [T6,T6_fwd] = water_reservoir.compute_tf(12,15,3,true);
    
    if verify_tf
        T6_analytical = water_reservoir.compute_tf_analytical(12,15,3,true);
        T6_matches = water_reservoir.tf_matches(T6,T6_analytical,verify_plot_step)
    end

%% PID 1 CONTROLLER - STEP, BODE, ROOT LOCUS
% Step response
    figure;
    step(T6);
    grid on
    legend('Kp=12, Ki=15, Kd=3');
% Bode plot
    figure;
    bode(T6_fwd);
    grid on
    legend('Kp=12, Ki=15, Kd=3');
% Root locus (using closed-loop)
    figure;
    rlocus(T6_fwd);
    grid on
    legend('Kp=12, Ki=15, Kd=3');

%% PID 2 CONTROLLER
% Calculate closed-loop transfer function with PID controller, Kp=9, Ki=15, Kd=2
    [T7,T7_fwd] = water_reservoir.compute_tf(9,15,2,true);
    
    if verify_tf
        T7_analytical = water_reservoir.compute_tf_analytical(9,15,2,true);
        T7_matches = water_reservoir.tf_matches(T7,T7_analytical,verify_plot_step)
    end
    
%% PID 2 CONTROLLER - STEP, BODE, ROOT LOCUS
% Step response
    figure;
    step(T7);
    grid on
    legend('Kp=9, Ki=15, Kd=2');
% Bode plot
    figure;
    bode(T7_fwd);
    grid on
    legend('Kp=9, Ki=15, Kd=2');
% Root locus (using closed-loop)
    figure;
    rlocus(T7_fwd);
    grid on
    legend('Kp=9, Ki=15, Kd=2');

%% PID 1 AND 2 - STEP
% Step response
    figure;
    hold on
    step(T6); step(T7);
    hold off
    grid on
    legend('Kp=12, Ki=15, Kd=3','Kp=9, Ki=15, Kd=2');
    
%% FINALIZE STEP INFO TABLE
% Extract step info
    T6_stepinfo = water_reservoir.extract_stepinfo_data(T6);
    step_info = [step_info; T6_stepinfo];
    T7_stepinfo = water_reservoir.extract_stepinfo_data(T7);
    step_info = [step_info; T7_stepinfo]
