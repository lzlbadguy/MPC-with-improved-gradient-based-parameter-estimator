function Fy0 = get_Fy0(alpha,mu,Fz)
b1=300;
b2 = 1.82;
b3 = 0.208;
b4 = 0;
b5 = -0.354;
b6 = 0.707;

Fz=Fz/1000;                         %transfer the unit of normal force Fz from N to kN

D = 0.8*mu*Fz;
C = 1.3;
BCD = b1*sin(b2*atan(b3*Fz))/10;
B = BCD/C/D; 
E = b4*Fz^2+b5*Fz+b6;

Fy0 = D*sin(C*atan(B*alpha-E*(B*alpha-atan(B*alpha))));

Fy0=Fy0*1000;                       %transfer the unit of lateral force Fy from kN to N
end

