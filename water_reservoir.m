%%--------------------------------------------------------------------
% AUTHOR:       Joyce Lai
% DATE:         2021 December
%
% DESCRIPTION
%
% This file includes functions that can be called via object-oriented
% syntax where the class is 'water_reservoir' and available functions
% are the following:
%
%       [T,T_fwd] = compute_tf(Kp,Ki,Kd,has_PID)
%       T = compute_tf_analytical(Kp,Ki,Kd,has_PID)
%       match = tf_matches(T_1,T_2,plot)
%       stepinfo_data = extract_stepinfo_data(T)
%
% The (closed-loop) transfer functions are specific to the water reservoir
% being modeled for a class project, and all constants besides PID are
% defined within the functions. Generally, the system's forward path
% consists of the PID controller, C(s), motor, P(s), and STH block, G(s),
% all in series. The system has unity feedback.
%
% Function descriptions are above each function definition.
%%--------------------------------------------------------------------
classdef water_reservoir
    methods (Static = true)
        %% COMPUTE TRANSFER FUNCTION %%
        
        % This function takes the general PID controller parameters and a
        % boolean, 'has_PID', and computes the closed-loop transfer
        % function, 'T', of the water reservoir system described above.
        % Additionally, it returns the open loop transfer function,
        % 'T_fwd' so it can be used for . All transfer functions are of
        % type tf. If 'has_PID' is false, the PID parameters will have no
        % affect on the returned values (the controller is replaced by a gain of 1).
        
        function [T,T_fwd] = compute_tf(Kp,Ki,Kd,has_PID)
            % Define constants
                J=0.01; b=0.1; Kt=0.1; Ke=0.01; R=1; L=0.5;
                A=0.5; Rf=0.5;
                Kf=1;
            % Define transfer functions
                s = tf('s');                
                if has_PID
                    C = (Kd*s^2+Kp*s+Ki)/s;
                else
                    C = 1;  % no controller
                end
                P = Kt/(J*L*s^2+(b*L+J*R)*s+b*R);
                G = (Rf/Kf)/(A*Rf*s+1);
                T_fwd = C*P*G;
                T = T_fwd/(1+T_fwd);
        end
        
        %% COMPUTE ANALYTICAL TRANSFER FUNCTION %%
        
        % This function takes the general PID controller parameters and a
        % boolean, 'has_PID', and computes the analytical closed-loop transfer
        % function, 'T'. Note that this 'T' typically differs from the 'T'
        % returned by the function above, 'compute_tf' since this one
        % includes pole-zero cancellation, i.e. will always be lower order
        % than the 'T' returned by 'compute_tf'. If 'has_PID' is false, the
        % PID parameters will have no affect on the returned values (the
        % controller will be replaced by a gain of 1).
        
        function T = compute_tf_analytical(Kp,Ki,Kd,has_PID)
            % Define constants
                J=0.01; b=0.1; Kt=0.1; Ke=0.01; R=1; L=0.5;
                A=0.5; Rf=0.5;
                Kf=1;
            % Define transfer function
                s = tf('s');
                if has_PID
                    % Define analytical transfer function coefficients with PID
                        a4=Kf*A*Rf*J*L;
                        a3=Kf*(A*Rf*(b*L+J*R)+J*L);
                        a2=Kf*A*Rf*b*R+Kf*b*L+Kf*J*R+Kt*Rf*Kd;
                        a1=Kf*b*R+Kt*Rf*Kp;
                        a0=Kt*Rf*Ki;
                    % Define analytical transfer function with PID
                        T_num = Kt*Rf*(Kd*s^2+Kp*s+Ki);
                        T_den = a4*s^4+a3*s^3+a2*s^2+a1*s+a0;
                        T = T_num/T_den;
                else
                    % Define analytical transfer function without PID
                        T = (Kt*Rf/Kf)/(A*Rf*J*L*s^3+(A*Rf*b*L+A*Rf*J*R+J*L)*s^2+(A*Rf*b*R+b*L+J*R)*s+b*R+Kt*Rf/Kf);
                end
        end

        %% DETERMINE WHETHER TRANSFER FUNCTIONS MATCH %%
        
        % This function takes two tf variables, 'T_1' and 'T_2', and a
        % boolean, 'plot', as input, and returns 'match', which is either 0
        % or 1 to indicate whether the transfer functions match. When
        % 'plot' is true, the function will plot the step responses of both
        % systems in one figure and vice versa if 'plot' is false. The
        % transfer functions are deemed "matching" if all step response
        % information from the function 'stepinfo' have a difference less
        % than tolerance, TOL = 0.001.
        
        function match = tf_matches(T_1,T_2,plot)
            TOL = 0.001;
            if abs(stepinfo(T_2).RiseTime - stepinfo(T_1).RiseTime) < TOL
                risetime_matches = 1;
            else
                risetime_matches = 0;
            end

            if abs(stepinfo(T_2).SettlingTime - stepinfo(T_1).SettlingTime) < TOL
                settlingtime_matches = 1;
            else
                settlingtime_matches = 0;
            end

            if abs(stepinfo(T_2).SettlingMin - stepinfo(T_1).SettlingMin) < TOL
                settlingmin_matches = 1;
            else
                settlingmin_matches = 0;
            end

            if abs(stepinfo(T_2).SettlingMax - stepinfo(T_1).SettlingMax) < TOL
                settlingmax_matches = 1;
            else
                settlingmax_matches = 0;
            end

            if abs(stepinfo(T_2).Overshoot - stepinfo(T_1).Overshoot) < TOL
                overshoot_matches = 1;
            else
                overshoot_matches = 0;
            end

            if abs(stepinfo(T_2).Undershoot - stepinfo(T_1).Undershoot) < TOL
                undershoot_matches = 1;
            else
                undershoot_matches = 0;
            end

            if abs(stepinfo(T_2).Peak - stepinfo(T_1).Peak) < TOL
                peak_matches = 1;
            else
                peak_matches = 0;
            end

            if abs(stepinfo(T_2).PeakTime - stepinfo(T_1).PeakTime) < TOL
                peaktime_matches = 1;
            else
                peaktime_matches = 0;
            end

            if risetime_matches & settlingtime_matches & settlingmin_matches & settlingmax_matches & overshoot_matches & undershoot_matches & peak_matches & peaktime_matches
                match = 1;
            else
                match = 0;
            end
            
            if plot
                figure;
                hold on
                step(T_1); step(T_2);
                grid on
                hold off
                title('System Verification Using Step Response');
                legend;
            end
        end
        
        %% EXTRACT STEP INFO %%
        
        % This function takes a tf (transfer function of type tf) and
        % returns a row vector with the rise time, settling time,
        % overshoot, ss value, and ss error, in that order.
        
        function stepinfo_data = extract_stepinfo_data(T)
            stepinfo_data = [stepinfo(T).RiseTime stepinfo(T).SettlingTime stepinfo(T).Overshoot];
            [y,t]=step(T); ss_val = y(end); ss_err = abs(1-ss_val);
            stepinfo_data = [stepinfo_data ss_val ss_err];
        end
    end
end