run('nominal_design.m');

%put the model in M-D form, Blocks is a description of the uncertainty
%structure
[M,Delta,Blocks] = lftdata(F);

%M has as inputs the four uncertain parameters and the setpoint so is 5x5
% we need the uncertainty block of M
M = M(1:4,1:4);


%CONSERVATIVE TEST
%check that the maximum singular value of M is below 1
figure(1); grid on;
sigma(M);
title('\sigma(M)');
xlabel('\omega');
ylabel('\sigma');

%STRUCTURED SINGULAR VALUE TEST
bounds=mussv(M(1:4,1:4),Blocks);

figure(2); grid on;
sigma(bounds);
title('structured singular value');
xlabel('\omega');
ylabel('\mu');
