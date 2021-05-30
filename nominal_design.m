clear variables;
addpath('lib');

%NOMINAL MODEL WITH STD = 1
[~,~,~,~,G] = ss_model(3, true);

% PERFORMANCE REQUIREMENT
s = tf('s');
wn = 15;
csi = 0.98;
L_so = 1/(1+2*csi*(s/wn) + (s/wn)^2);
Wpinv = 1/(1+L_so);
Wp = 1/Wpinv;

% CONTROL REQUIRMENT
epsu = 0.01;
wbu = 1;
Mu = 2;
Wqinv = (epsu * s + wbu)/(s+wbu/Mu);
Wq = 1/Wqinv;
%--------------------------------------------------------------------------
%GENSS MODEL
Rp = tunablePID('Rp','pid');
Rphi = tunablePID('Rphi','p');

integrator = 1/s;

%creiamo input e output dei blocchi
Rphi.u = 'ephi';
Rphi.y = 'p_0';
Rp.u = 'e_p';
Rp.y='\delta_{lat}';
G.u='\delta_{lat}';
G.y='p';
integrator.u='p';
integrator.y='\phi';

%specify summing junctions
sum1 = sumblk('ephi = \phi_0 - \phi');
sum2 = sumblk('e_p = p_0 - p');

%create genss model
T0 = minreal(connect(Rphi,Rp,G,integrator,sum1,sum2,{'\phi_0'},{'ephi','\delta_{lat}','\phi'}));

rng('default');

N_TESTS = 10;
Req = [
    TuningGoal.WeightedGain('\phi_0','ephi',Wp, 1);
    TuningGoal.WeightedGain('\phi_0','\delta_{lat}',Wq, 1)
];

opt = systuneOptions('RandomStart',N_TESTS, 'SoftTol', 1e-7, 'Display', 'iter');
[T, J, ~] = systune(T0,Req, opt);

figure(1);
t1=1; t2 = 2; 
step(T*10*(1-2*exp(-t1*s) + exp(-s*t2)));


[A,B,C,D,~] = ss_model(3, false);
G = uss(A,B,C,D);

%GENSS MODEL
Rp = T.blocks.Rp;
Rphi = T.blocks.Rphi;

integrator = 1/s;

%creiamo input e output dei blocchi
Rphi.u = 'ephi';
Rphi.y = 'p_0';
Rp.u = 'e_p';
Rp.y='\delta_{lat}';
G.u='\delta_{lat}';
G.y='p';
integrator.u='p';
integrator.y='\phi';

%specify summing junctions
sum1 = sumblk('ephi = \phi_0 - \phi');
sum2 = sumblk('e_p = p_0 - p');

%create genss model
T = minreal(connect(Rphi,Rp,G,integrator,sum1,sum2,{'\phi_0'},{'ephi','\delta_{lat}','\phi'}));
Q = getIOTransfer(T, '\phi_0','\delta_{lat}');
S = getIOTransfer(T, '\phi_0','ephi');

figure(2);
bodemag(S,'b-',Wpinv,'r-');

figure(3);
bodemag(Q,'b-',Wqinv,'r-');

figure(4);
step(T*10*(1-2*exp(-t1*s) + exp(-s*t2)));







