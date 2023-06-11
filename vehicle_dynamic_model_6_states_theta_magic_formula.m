function x_up = vehicle_dynamic_model_6_states_theta_magic_formula(x,u,Ts,theta)


lf=1.2;             % lf is the distance from the center of mass to the vehicle's front axis                                
lr=1.6;             % lr is the distance from the center of mass to the vehicle's rear axis
m = 1575;           % m is the mass of the vehicle
Iz = 2875;          % Iz is the moment of inertia for the vehicle
g = 9.8;            % g is the gravitation acceleration
mu=0.9;             % mu is the adhesion coefficient of this road section

% The term cos(phi) is absent since we don't know the road grade and phi is small so cos(phi) is close to 1.
Fzf=m*g*lr/(lf+lr);     %Normal force applied to the front wheels
Fzr=m*g*lf/(lf+lr);     %Normal force applied to the rear wheels

af = u(2) - (x(2)+lf*x(3))./x(1);  % Calculate the front wheels' sliping angle
ar = -(x(2)-lr*x(3))./x(1);        % Calculate the rear wheels' sliping angle

Fyf = get_Fy0(af,mu,Fzf);         % Calculate lateral force applied to the front wheels
Fyr = get_Fy0(ar,mu,Fzr);         % Calculate lateral force applied to the rear wheels

a_phi=theta;                     % theta is the variable estimated by the improved gradient method outside the MPC
                                  % a(phi) = theta

% x(1)-vx(k),x(2)-vy(k),x(3)-psi_dot(k),x(4)-psi(k),x(5)-X(k),x(6)-Y(k)                                 
f1 = x(3).*x(2) + u(1)-a_phi*cos(x(4));             % Calculate vx_dot
f2 = -x(3).*x(1) + 1/m*(Fyf+Fyr)+a_phi*sin(x(4));   % Calculate vy_dot
f3 = 1/Iz*(lf*Fyf-lr*Fyr);                          % Calculate psi_ddot
f4 = x(3);                                          % Calculate psi_dot
f5 = x(1).*cos(x(4))-x(2).*sin(x(4));               % Calculate Xdot
f6 = x(1).*sin(x(4))+x(2).*cos(x(4));               % Calculate Ydot

xdot=[f1;f2;f3;f4;f5;f6];
    
x_up=x+xdot*Ts;                                     % x(k+1)=x(k)+xdot(k)*Ts using Euler Update law to update the prediction model 
end

