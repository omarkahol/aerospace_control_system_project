run('nominal_design.m');

G_n = tf(G.NominalValue);

G_array = usample(G,100);

[~,Info] = ucover(G_array,G_n,1);

figure(1); hold on; grid on;
bodemag(tf(F),'b-');
bodemag(1/Info.W1,'r--');
legend("F(j \omega)","W_{inv}(j \omega)");