clear variables;
addpath('lib');

%NOMINAL MODEL WITH STD = 1
[~,~,~,~,G] = ss_model(3, true);

% PERFORMANCE REQUIREMENT
s = tf('s');
wn = 15;
csi = 0.99;
L_so = 1/(1+2*csi*(s/wn) + (s/wn)^2);
Wpinv = 1/(1+L_so);
Wp = 1/Wpinv;

%--------------------------------------------------------------------------
%GENSS MODEL
Rp = tunablePID('Rp','pid');
Rphi = tunablePID('Rphi','p');
integrator = 1/s;

Wqinv = tf(1);
Wq = 1/Wqinv;
%creiamo input e output dei blocchi
Wp.u = 'e_{\phi}';
Wp.y = 'e_{perf}';

Rphi.u = 'e_{\phi}';
Rphi.y = 'p_0';
Rp.u = 'e_p';
Rp.y='\delta_{lat}';
G.u='\delta_{lat}';
G.y='p';
integrator.u='p';
integrator.y='\phi';
Wq.u='\delta_{lat}';
Wq.y='e_{control}';

%specify summing junctions
sum1 = sumblk('e_{\phi} = \phi_0 - \phi');
sum2 = sumblk('e_p = p_0 - p');

%create genss model
T0 = minreal(connect(Wp,Wq,Rphi,Rp,G,integrator,sum1,sum2,{'\phi_0'},{'\phi','e_{perf}','e_{control}'}));

rng('default')
opt = hinfstructOptions('Display','final','RandomStart',5);
T = hinfstruct(T0,opt);

figure(1);
step(T*10*(1-exp(-2*s)));

Rp = getBlockValue(T,'Rp');
Rphi = getBlockValue(T,'Rphi');
%----------------------------------------------------------------------------------------------------

Rphi.u = 'e_{\phi}';
Rphi.y = 'p_0';
Rp.u = 'e_p';
Rp.y='\delta_{lat}';
G.u='\delta_{lat}';
G.y='p';
integrator.u='p';
integrator.y='\phi';
sum1 = sumblk('e_{\phi} = \phi_0 - \phi');
sum2 = sumblk('e_p = p_0 - p');
T_F = minreal(connect(Rphi,Rp,G,integrator,sum1,sum2,{'\phi_0'},{'\phi'}));
T_Q = minreal(connect(Rphi,Rp,G,integrator,sum1,sum2,{'\phi_0'},{'\delta_{lat}'}));

figure(2);
bodemag(T_F,'k-',1-T_F,'b-',Wpinv,'r--');

figure(3);
bodemag(T_Q,'b-',Wqinv,'r--');