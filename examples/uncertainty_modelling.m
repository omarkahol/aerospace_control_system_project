gamma = ureal('gamma',2,'Perc',30);  % uncertain gain

tau = ureal('tau',1,'Perc',30);      % uncertain time-constant

xi = ureal('xi',0.25,'Perc',70);      % uncertain time-constant

% parameters of second order mode
wn = 50; 

% uncertain plant
P = tf(gamma,[tau 1]) * tf(wn^2,[1 2*xi*wn wn^2]);

figure(1),step(P,5)

figure(2),bode(P),grid
%% 

Parray = usample(P,60);

% Pn = P.NominalValue; % Case in which the nominal model is the mean model

Pn=tf(2,[1 1]); % Case in which the nominal model is a reduced order model

Wt=tf(0.4*[1 1],[1/10 1]);

figure(3), bodemag((Pn-Parray)/Pn,Wt,'r')
%% 

[P,Info] = ucover(Parray,Pn,1);

figure(4),bodemag((Pn-Parray)/Pn,Wt,'r',Info.W1,'g')
%% 

[P,Info] = ucover(Parray,Pn,5);

figure(5),bodemag((Pn-Parray)/Pn,Wt,'r',Info.W1,'g')