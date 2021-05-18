clear variables;

%NOMINAL MODEL WITH STD = 1
[A,B,C,D] = ss_model(1, true);


system = ss(A,B,C,D);
plant = tf(system);

plant

bode(plant);

