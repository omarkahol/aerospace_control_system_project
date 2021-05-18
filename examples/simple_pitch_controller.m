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
G_n = getNominal(G);

%MODELLO INCERTO
G_array = usample(G,50);
[~, Info] = ucover(G_array, G_n, 5); % CALCOLO DEL PESO DELL'INCERTEZZA

%PLOT DEL PESO DELL'INCERTEZZA
figure(1);
hold on;
bodemag((G_array-G_n)/G_n,'b', Info.W1,'r');
hold off;

%CONTROLLER PD
R = tunablePID('c','pd');

%NOMINAL TRANSFER FUNCTIONS
L = R*G_n;
S = 1/(1+L);
F = L/(1+L);

%TRANSFER FUNCTION PER IL PESO DELLA NOMINAL STABILITY
M = 3;
omb = 12;
A = 1e-5;
Wpinv = (s+omb*A)/(s/M + omb);
Wp = 1/Wpinv;

%H_INF SYNTHESIS
opt = hinfstructOptions('Display','final','RandomStart',20);
K = hinfstruct([S*Wp,F*Info.W1],opt);

R = pid(K.Blocks.c); % RECOVERY DEL CONTROLLORE

%SIMULAZIONE DEL MODELLO INCERTO
L = G*R;
F = L/(1+L);
step(F,10);