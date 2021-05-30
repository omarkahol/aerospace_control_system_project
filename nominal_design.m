clear variables;
addpath('lib');

%NOMINAL MODEL WITH STD = 1
[~,~,~,~,G] = ss_model(3, true);

% PERFORMANCE REQUIREMENT
s = tf('s');
wn = 12;
csi = 0.96;
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
T0 = minreal(connect(Rphi,Rp,G,integrator,sum1,sum2,{'\phi_0'},{'ephi','\delta_{lat}'}));

rng('default');

N_TESTS = 10;
Req = [
    TuningGoal.WeightedGain('\phi_0','ephi',Wp, 1);
    TuningGoal.WeightedGain('\phi_0','\delta_{lat}',Wq, 1)
];

opt = systuneOptions('RandomStart',N_TESTS, 'SoftTol', 1e-7, 'Display', 'iter');
[T, J, ~] = systune(T0,Req, opt);
figure(1);
step(T);


