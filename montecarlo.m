%run('nominal_design.m');

%% Pre Allocamento Variabili
samples=1000;

F_st_mc=zeros(1,samples); 
F_rt_mc=zeros(1,samples);

delta_lat_mc = zeros(1,samples);

Gm=zeros(1,samples);
Pm=zeros(1,samples);
Sm=zeros(1,samples);
    
   samples=1000;
   
for i=1:samples
    %% Complementary Sensitivity
    
    F_0=usample(F,1);                                                             % prendo un campione incerto della Complementary sens
    
    [y_F,t_F]=step(F_0,linspace(0,10,1000));                                        % faccio lo step
    
    step_info_F=stepinfo(y_F,t_F,1);                                             % ne estraggo i valori 
    
    F_st_mc(i) = step_info_F.SettlingTime;                                         % assegno i diversi valori a una matrice di zeri
    F_rt_mc(i) = step_info_F.RiseTime;
    
 %% Control effort   
    Q_0=usample(Q,1);
    
    [y_Q,t_Q]=step(Q_0*10*(1-2*exp(-t1*s)+exp(-s*t2)),linspace(0,10,1000));
    
    delta_lat_mc(i)=max(abs(y_Q));
   
 %% Loop transfer function F=L/(1+L),S=1/(1+L) -----> L=F/S
  S_0=usample(S,1);
  
 [Gm(i),Pm(i)]=margin(F_0/S_0);
 
  Sm(i)=1/getPeakGain(S_0);                                                      %stability margin 1/Max_S
 
 
end

%% Histograms
figure(1),histogram(F_st_mc,100),grid,title 'Settling Time F';
figure(2),histogram(F_rt_mc,100),grid,title 'Rise Time F';

figure(3),histogram(Gm,100),grid,title 'Gain Margin';
figure(4),histogram(Pm,100),grid,title 'Phase Margin';
figure(5),histogram(Sm,100),grid,title 'Stability Margin';

figure(6),histogram(delta_lat_mc,100),grid,title 'Delta Lateral';





