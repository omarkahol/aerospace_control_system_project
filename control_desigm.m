clear variables;

%NOMINAL MODEL WITH STD = 1
[A,B,~,~] = ss_model(1, true);
s = zpk('s');

%G_P 
C = [0,1,0];
G_p = minreal(tf(ss(A,B,C,0)));

%G_PHI
C = [1,0,0];
G_phi = minreal(tf(ss(A,B,C,0)));

%CONTROLLERS
R_p = tunablePID('C1', 'pid');
R_phi = tunableGain('C2', 'pid');

%TRANSFER FUNCTION
BLOCK_1 = 1-(G_p*R_p/(1+G_p*R_p));
BLOCK_2 = R_phi*R_p*G_phi;

F = BLOCK_1*BLOCK_2 / (1 + BLOCK_1*BLOCK_2);
S = 1 / (1 + BLOCK_1*BLOCK_2);

omb = 11;
A = 0.01;
M = 10;
Wp = ((s/M) + omb)/(s+omb*A);
P = augw(BLOCK_1*BLOCK_2,Wp,[],[]);
opt = hinfstructOptions('Display','final','RandomStart',50);
K = hinfstruct(P,opt);
