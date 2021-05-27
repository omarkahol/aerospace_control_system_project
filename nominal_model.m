clear variables;

%NOMINAL MODEL WITH STD = 1
[A,B,~,~] = ss_model(3, true);

C_p = [0,1,0];
D = 0;
G_p = tf(ss(A,B,C_p,D));

s = tf('s');

integrator = 1/s;

% PESO SULLA SENSITIVITY FUNCTION
M = 5;
omb = 11;
A = 1e-1;
Wpinv = (s+omb*A)/(s/M + omb);
Wp = 1/Wpinv;

% PESO SULLA CONTROL EFFORT
Wqinv = tf(1);
Wq = 1/Wqinv;

%GENSS MODEL
Rp = tunablePID('Rp','pid');
Rphi = tunablePID('Rphi','p');

%creiamo input e output dei blocchi
Wp.u = 'e_phi';
Wp.y = 'e_ws';
Wq.u = 'delta_lat';
Wq.y = 'eq';
Rphi.u = 'e_phi';
Rphi.y = 'p_0';
Rp.u = 'e_p';
Rp.y='delta_lat';
G_p.u='delta_lat';
G_p.y='p';
integrator.u='p';
integrator.y='phi';

%specify summing junctions
sum1 = sumblk('e_phi = phi_0 - phi');
sum2 = sumblk('e_p = p_0 - p');

%create genss model
T0 = connect(Wp,Wq,Rphi,Rp,G_p,integrator,sum1,sum2,{'phi_0'},{'phi','e_ws','eq'});

rng('default')
opt = hinfstructOptions('Display','final','RandomStart',10);
T = hinfstruct(T0,opt);

Rp = getBlockValue(T,'Rp');
Rphi = getBlockValue(T,'Rphi');


L = integrator*G_p*Rp*(1- G_p*Rp/(1+G_p*Rp))*Rphi;

figure(1);
step(T,linspace(0,0.1,1000));

figure(2);
bodemag(1/(1+L), 'b-', Wpinv, 'r--');
