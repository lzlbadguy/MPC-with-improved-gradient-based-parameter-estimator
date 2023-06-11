function vx_up=get_vx_up(xlast,u,Ts,theta_hat)    
    vx_dot_last=xlast(3)*xlast(2)+u(1)-theta_hat*cos(xlast(4)); %vx_hat_dot(k)=psi_dot(k)*vy(k)+ax(k)-a(phi_hat(k))*cos(psi(k));
    vx_up= xlast(1)+vx_dot_last*Ts;                             %vx_hat(k+1)=vx_hat(k)+vx_hat_dot(k)*Ts;
end