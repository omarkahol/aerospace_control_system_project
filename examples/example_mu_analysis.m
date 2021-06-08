%
%   Comparison of different approaches for rubust stability analysis
%   (including mu-analysis)
%


% Definition of plant model and complementary sensitivity
s=tf('s');

wI=(s+0.2)/(0.5*s+1);

KG=1/s*[-0.0015 0; 0 -0.075]*[-87.8 1.4; -108.2 -1.4];

F=minreal(inv(eye(2,2)+KG)*KG);

M=minreal(wI*F);

% Robust stability using test for complex uncertainty
figure(1)
sigma(F,1/wI), grid;

figure(2)
sigma(M), grid;

% Robust stability using mu
omega=logspace(-3,2,500);
bounds=mussv(frd(M,omega),[1 0; 1 0]);

figure(3)
sigma(bounds), grid
