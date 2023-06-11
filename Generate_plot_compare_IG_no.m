clc;
clear;
load('nominal_mpc_IG_lane_change_slope_compare_no.mat');
load('nominal_mpc_lane_change_slope_no_adapt.mat');
figure
subplot(2,2,1)
plot(th,xh(1,:),'linewidth',2);
hold on;
plot(th,xh_no(1,:),'g--','linewidth',2);
xlabel('T (s)');
ylabel('$v_{x}$ (m/s)','interpreter','latex','FontSize',15,'Rotation',90);
legend('IG','No');
subplot(2,2,2)
plot(th,xh(6,:),'linewidth',2);
hold on;
plot(th,xh_no(6,:),'g--','linewidth',2);
xlabel('T (s)');
ylabel('$Y$ (m)','interpreter','latex','FontSize',15,'Rotation',90);
subplot(2,2,3)
plot(th,uh(1,:),'linewidth',2);
hold on;
plot(th,uh_no(1,:),'g--','linewidth',2);
xlabel('T (s)');
ylabel('$a_{x} (m/s^2)$','interpreter','latex','FontSize',15,'Rotation',90);
subplot(2,2,4)
plot(th,uh(2,:),'linewidth',2);
hold on;
plot(th,uh_no(2,:),'g--','linewidth',2);
xlabel('T (s)');
ylabel('$\delta_{f}$ (rad)','interpreter','latex','FontSize',15,'Rotation',90);

figure;
plot(th,xh(4,:)*180/pi,'linewidth',2);
hold on;
plot(th,xh_no(4,:)*180/pi,'g--','linewidth',2);
xlabel('T (s)');
ylabel('$\psi^{\circ}$','interpreter','latex','FontSize',15,'Rotation',0);
legend('IG','No');

figure;
plot(th,-theta_hat_h,'linewidth',2);
hold on;
plot(th,-theta_h,'--','linewidth',2);
hold on;
plot(th,-theta_hat_h_no,'g--','linewidth',2);
xlabel('T (s)');
ylabel('$a(\hat\phi)$ ','interpreter','latex','FontSize',15,'Rotation',90);
legend('$a(\hat\phi)$','$a(\phi)$','No adapt','interpreter','latex','FontSize',15);