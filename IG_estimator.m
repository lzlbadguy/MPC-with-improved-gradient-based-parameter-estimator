    zeta=cos(xlast(4))*Ts_sim;                          % use zeta to simplifified the equation for the parameter adaptation law. 
    vx_hat_up=get_vx_up(xlast,u,Ts_sim,theta_hat);      % vx_hat(k+1)=vx_hat_up;
    vx_up=x(1);                                         % vx(k+1) = vx_up;
    theta_hat_up=theta_hat+alpha*(vx_hat_up-vx_up)*zeta/(1+alpha*zeta^2); % Calculate a(phi(k+1))=theta_hat(k+1);
    theta_hat=theta_hat_up;                             % Update a(phi(k))