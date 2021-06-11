close all;
clear variables;
addpath('lib');

%NOMINAL MODEL WITH STD = 1
[A,B,C_p,D,G_p] = ss_model(3, true);

poles = eig(G_p);
zeros = eig(1/G_p);

figure(1); hold on; grid on; axis on;

p=plot(real(poles),imag(poles), 'r*', 'LineWidth', 3);
z=plot(real(zeros),imag(zeros), 'bo', 'LineWidth', 3);

legend([p,z],'Poles', 'Zeros');
hold off;

%UNCERTAIN MODEL
[A,B,C_p,D,G_p] = ss_model(3, false);

G = uss(A,B,C_p,D);
samples = 100;
G_array = tf(usample(G, samples));

figure(2); hold on; grid on; axis on;
for i = 1:samples
    G_val=G_array(:,:,i,1);
    poles_u = eig(G_val);
    zeros_u = eig(1/G_val);

    p_u=plot(real(poles_u),imag(poles_u), 'r*', 'LineWidth', 1);
    z_u=plot(real(zeros_u),imag(zeros_u), 'bo', 'LineWidth', 1);
end
p=plot(real(poles),imag(poles), 'k*');
z=plot(real(zeros),imag(zeros), 'ko');

legend([p_u,z_u, p, z],'Uncertain Poles', 'Uncertain Zeros', 'Poles', 'Zeros');
hold off;

figure(3); hold on; grid on;
bode(G,'b-');
bode(G_p,'r--');
legend('Uncertain Model', 'Nominal Model');






