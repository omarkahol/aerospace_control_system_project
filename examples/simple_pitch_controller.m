% ESEMPIO COMPLETO DI H_INF SINTHESYS
% DATA at https://ctms.engin.umich.edu/CTMS/index.php?example=AircraftPitch&section=SystemModeling


clear variables;

s = zpk('s');

%PARAMETRI DELLA TRANSFER FUNCTION
b1 = ureal('b1',1.51, 'Percentage', 2.3);
b2 = ureal('b2',0.1774, 'Percentage', 2.1);
b3 = 1;
b4 = ureal('b4',0.739, 'Percentage', 11);
b5 = ureal('b5',0.921, 'Percentage', 7);

% funzione di trasferimento da elevatore a pitch
G = (b1*s+b2)/(b3*s^3 + b4*s^2 + b5*s);

%nominal model
G_n = tf(getNominal(G));

%MODELLO INCERTO
G_array = usample(G,50);
[~, Info] = ucover(G_array, G_n, 5); % CALCOLO DEL PESO DELL'INCERTEZZA


%CONTROLLER PD
R = tunablePID('c','pidf');

%TRANSFER FUNCTION PER IL PESO DELLA NOMINAL STABILITY
M = 1;
omb = 30;
A = 0.001;
Wpinv = (s+omb*A)/(s/M + omb);
Wp = 1/Wpinv;

%CONTROL EFFORT TRANSFER FUNCTION
M = 1;
omb = 30;
A = 0.001;
Wqinv=10*(0.001*s+1)/(100*s+1);
Wq=1/Wqinv; 

% GENSS MODEL
R.u = 'e';
R.y = 'u';
Wp.u='e';
Wp.y='ew';
Wq.u = 'u';
Wq.y = 'eq';
Wpinv.u='nw';
Wpinv.y='n';
sum2 = sumblk('yn = y + n');

G_n.u = 'u';
G_n.y = 'y';
sum1 = sumblk('e = y0 - yn');
T0 = connect(Wp,Wq,Wpinv,R,G_n,sum1,sum2,{'y0', 'nw'},{'y','ew','eq'});

rng(0)
opt = hinfstructOptions('Display','final','RandomStart',20);
T = hinfstruct(T0,opt);
R = pid(T.Blocks.c);

figure(1);
bodemag(R/(1+G*R),'b-',Wqinv,'r--',{1e-5,1e+5});

figure(2);
bodemag(1/(1+G*R),'b-',Wpinv,'r--',{1e-5,1e+5});

P = augw(G_n,Wp,Wq,[]);

% Structured synthesis
K0 = tunablePID('C','pid');
opt = hinfstructOptions('Display','final','RandomStart',5);
K = hinfstruct(P,K0,opt);

figure(3),bode(1/(1+G*K),Wpinv),grid,legend('S','1/W1')
figure(4),bode(K/(1+G*K),Wqinv),grid,legend('Q','1/W2')