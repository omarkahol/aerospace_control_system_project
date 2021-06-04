
clear variables;
addpath('lib');

%NOMINAL MODEL WITH STD = 1
%creo il modello nominale (quindi passando true a ss_model)
[~,~,~,~,G] = ss_model(1, true);

% PERFORMANCE REQUIREMENT
s = tf('s'); %variabile di laplace s
wn = 12; %crossover frequency
csi = 0.95; %damping ratio
F_so =1/(1+2*csi*(s/wn) + (s/wn)^2); %open loop transfer function --> obiettivo

%pesi
Wpinv = 1-F_so;
Wp = 1/Wpinv;

% CONTROL REQUIRMENT
epsu = 0.01; %valore a frequenze elevate
wbu = 1; % crossover frequency
Mu = 2; %valore a frequenze basse 
%psei
Wqinv =0.1*(s+100*wn)/(s+10*wn);
Wq = 1/Wqinv;


%--------------------------------------------------------------------------
%GENSS MODEL
%--------------------------------------------------------------------------
Rp = tunablePID('Rp','pi'); %controller PI optional
Rp2= tunablePID('Rp2','pd'); % controlle D optional
Rp2.Kp.Value=0;
Rp2.Kp.Free=false;
% Rp2.Kd.Value=0;
% Rp2.Kd.Free=false;
Rp2.Tf.Value = 0.08;  
Rp2.Tf.Free = false; 
Rphi = tunablePID('Rphi','p'); %controller P


%blocco integratore
integrator = 1/s;

%creiamo input e output dei blocchi

%controller phi
Rphi.u = 'ephi'; 
Rphi.y = 'p_0';

%controller p
Rp.u = 'e_p';
Rp.y='\delta_{prelat}'; 

% Controller p2
Rp2.u='p';
Rp2.y='y_{Rp2}';

%plant
G.u='\delta_{lat}';
G.y='p';

%uscita
integrator.u='p';
integrator.y='\phi';

%blocchi somma
sum1 = sumblk('ephi = \phi_0 - \phi');
sum2 = sumblk('e_p = p_0 - p');
sum3 = sumblk('\delta_{lat}= \delta_{prelat}-y_{Rp2}');

%modello gens
T0 = minreal(connect(Rphi,Rp,Rp2,G,integrator,sum1,sum2,sum3,{'\phi_0'},{'ephi','\delta_{lat}','\phi'})); % optional

%--------------------------------------------------------------------------
% H INFINITY SYNTHESIS
%--------------------------------------------------------------------------
% LANCIAMO N_TEST CICLI DI OTTIMIZZAZIONE 

rng('default'); %inizializza il random number generator

N_TESTS = 10; %numero di dati iniziali random
%requisiti
Req = [ 
    TuningGoal.WeightedGain('\phi_0','ephi',Wp, 1);
    TuningGoal.WeightedGain('\phi_0','\delta_{lat}',Wq, 1)
];

opt = systuneOptions('RandomStart',N_TESTS, 'SoftTol', 1e-7, 'Display', 'iter');
[T, J, ~] = systune(T0,Req, opt); 


[A,B,C,D,~] = ss_model(3, false);
G = uss(A,B,C,D);

%--------------------------------------------------------------------------
% MODELLO INCERTO
%-------------------------------------------------------------------------- 

%RIDEFINISCO I BLOCCHI
Rp = T.blocks.Rp; %recupero i valori di Rp
Rphi = T.blocks.Rphi; %recupero i valori di Rphi
Rp2=T.blocks.Rp2;
integrator = 1/s;
Rphi.u = 'ephi';
Rphi.y = 'p_0';
Rp.u = 'e_p';

Rp.y='\delta_{prelat}'; 
Rp2.u='p';
Rp2.y= 'y_{Rp2}'; 
G.u='\delta_{lat}';
G.y='p';
integrator.u='p';
integrator.y='\phi';
sum1 = sumblk('ephi = \phi_0 - \phi');
sum2 = sumblk('e_p = p_0 - p');
sum3 = sumblk('\delta_{lat}= \delta_{prelat}-y_{Rp2}'); % optional

% MODELLO GENSS INCERTO

T = minreal(connect(Rphi,Rp,Rp2,G,integrator,sum1,sum2,sum3,{'\phi_0'},{'ephi','\delta_{lat}','\phi'})); 
Q = getIOTransfer(T, '\phi_0','\delta_{lat}'); % SENSITIVITY FUNCTION
S = getIOTransfer(T, '\phi_0','ephi'); %CONTROL EFFORT TRANSFER FUNCTION

% PLOT DELLA SENSITIVITY FUNCTION
figure(1);
bodemag(S,'b-',Wpinv,'r-');

% PLOT DELLA CONTROL EFFORT
figure(2);
bodemag(Q,'b-',Wqinv,'r-');

% RISPOSTA AL DOPPIO STEP
figure(3);
t1 = 1; t2 = 2;
step(T*10*(1-2*exp(-t1*s) + exp(-s*t2)));







