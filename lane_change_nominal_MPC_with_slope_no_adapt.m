clc;
clear;
gamma = 0.006;
%% Casadi define
import casadi.*
Ts_p = 0.2;
N = 15;
n_states = 6;
n_controls = 2;
n_particle = 200;

P = SX.sym('P',n_states + 3);
U = SX.sym('U',n_controls,N);
X = SX.sym('X',n_states,(N+1));

X(:,1) = P(1:6);
Theta = P(end);

for k = 1:N
%     vehicle_dynamic_model_6_states_theta_magic_formula(x,u,Ts,theta)
    X(:,k+1) = vehicle_dynamic_model_6_states_theta_magic_formula(X(:,k),U(:,k),Ts_p,Theta);
end

obj = 0; % Objective function
g = [];  % constraints vector

Q=1000;
R=1;
M=30;
% compute objective
for k=1:N
    obj = obj+(X(1,k+1)-P(7))'*Q*(X(1,k+1)-P(7))/25^2;
    obj = obj+(X(6,k+1)-P(8))'*Q*(X(6,k+1)-P(8))/3.5^2;
    obj = obj+(U(1,k))'*R*(U(1,k))/(2*sqrt(2))^2;
    obj = obj+(U(2,k))'*R*(U(2,k))/(pi/6)^2;
    if k>1
        obj = obj+(U(1,k)-U(1,k-1))'*R*(U(1,k)-U(1,k-1))/(1.5)^2;
        obj = obj+(U(2,k)-U(2,k-1))'*R*(U(2,k)-U(2,k-1))/(pi/12)^2;
    end
end

% compute constraints
for k = 1:N+1   % box constraints due to the map margins
    g = [g ; X(1,k)];   %state vx
    g = [g ; X(2,k)];   %state vy
    g = [g ; X(6,k)];   %state Y
    if k>1 && k<N+1
        g = [g ; (U(1,k)-U(1,k-1))]; 
        g = [g ; (U(2,k)-U(2,k-1))]; 
    end
end

% make the decision variables one column vector
OPT_variables = reshape(U,2*N,1);
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

args = struct;
ubg=[];
lbg=[];

for k=1:N+1
    lbg=[lbg;0];
    lbg=[lbg;-5];
    lbg=[lbg;-2];
    ubg=[ubg;30];
    ubg=[ubg;5];
    ubg=[ubg;2];
        if k>1 && k<N+1
            lbg=[lbg;-3*Ts_p];
            lbg=[lbg;-pi/36*Ts_p];
            ubg=[ubg;1.5*Ts_p];
            ubg=[ubg;pi/36*Ts_p];
        end
end
args.ubg=ubg;
args.lbg=lbg;

ubx=[];
lbx=[];

for k=1:N
   ubx=[ubx;4]; 
   ubx=[ubx;pi/18]; 
   lbx=[lbx;-4]; 
   lbx=[lbx;-pi/18]; 
end

args.ubx=ubx;
args.lbx=lbx;
%% THE SIMULATION LOOP SHOULD START FROM HERE
t = 0;
x = [20; 0; 0; 0; 0; 1.75];    % initial condition.
xs = [30; -1.75]; % Reference posture.
u=[0;0];

th=[];
xh=[];
uh=[];
th=[th,t];
xh=[xh,x];
uh=[uh,u];
simtime=15;
Ts_sim=0.1;
u0 = zeros(N,2);

% phi_now=pi/18*sin(4*pi/15*t);
phi_now=pi/72;
phi_true=[phi_now];
theta=9.8*(sin(phi_now)+gamma*cos(phi_now));
theta_h =[theta];
theta_hat=-2;
theta_hat_h=[theta_hat];
alpha=500;

for i=1:(simtime/Ts_sim)
    tic;
    if i>(simtime/Ts_sim)/2
        xs = [20; 1.75];
    end
    
    args.p   = [x;xs;theta_hat];
    args.x0 = reshape(u0',2*N,1);
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);   
    uout = reshape(full(sol.x)',2,N)';
    ulast=u;
    u=reshape(uout(1,:),2,1);
    xlast=x;
%     vehicle_dynamic_model_6_states_phi_magic_formula(x,u,Ts,phi);
    x = vehicle_dynamic_model_6_states_phi_magic_formula(x,u,Ts_sim,phi_now);   
    
    t=t+Ts_sim;
%     phi_now = pi/18*sin(4*pi/15*t);
    if t<5
        phi_now =pi/72;
    elseif t>=5&&t<10
        phi_now =-pi/36;
    elseif t>=10
        phi_now =pi/18; 
    end


%     zeta=cos(xlast(4))*Ts_sim;
%     vx_hat_up=get_vx_up(xlast,u,Ts_sim,theta_hat);
%     vx_up=x(1);
%     theta_hat_up=theta_hat+alpha*(vx_up-vx_hat_up)*zeta/(1+alpha*zeta^2);
%     theta_hat=theta_hat_up;
    theta_hat_h=[theta_hat_h,theta_hat];
    
    theta=9.8*(sin(phi_now)+gamma*cos(phi_now));
    theta_h =[theta_h,theta];
    
    u0 = [uout(2:N,:);uout(N,:)];
    th=[th,t];
    xh=[xh,x];
    uh=[uh,u];
    
    sample_time(i)=toc;
end
Simulation_run_time=sum(sample_time)

figure
subplot(2,2,1)
plot(th,xh(1,:),'linewidth',2);
xlabel('T (s)');
ylabel('$v_{x}$ (m/s)','interpreter','latex','FontSize',15,'Rotation',90);
subplot(2,2,2)
plot(th,xh(6,:),'linewidth',2);
xlabel('T (s)');
ylabel('$Y$ (m)','interpreter','latex','FontSize',15,'Rotation',90);
subplot(2,2,3)
plot(th,uh(1,:),'linewidth',2);
xlabel('T (s)');
ylabel('$a_{x} (m/s^2)$','interpreter','latex','FontSize',15,'Rotation',90);
subplot(2,2,4)
plot(th,uh(2,:),'linewidth',2);
xlabel('T (s)');
ylabel('$\delta_{f}$ (rad)','interpreter','latex','FontSize',15,'Rotation',90);

figure;
plot(th,xh(2,:),'linewidth',2);
xlabel('T (s)');
ylabel('$v_{y}$ (m/s)','interpreter','latex','FontSize',15,'Rotation',0);

figure;
plot(th,theta_hat_h,'linewidth',2);
hold on;
plot(th,-theta_h,'--','linewidth',2);
xlabel('T (s)');
ylabel('$a(\hat\phi)$ ','interpreter','latex','FontSize',15,'Rotation',90);
legend('$a(\hat\phi)$','$a(\phi)$','interpreter','latex','FontSize',15);

figure;
plot(1:length(sample_time),sample_time,'linewidth',2);
xlabel('Sampling Instance');
ylabel('Optimization Time(s)');