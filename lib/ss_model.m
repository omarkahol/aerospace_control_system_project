function [A,B,C_p,D, G_p] = ss_model(sigma, nominal)
%STATE-SPACE_TRANSFER-FUNCTION MODEL compute the
%state space representation of the system
%   INPUT
%       sigma ==> standard deviation of the uncertainty
%       nominal ==> if true, return the nominal model
%   
%   OUTPUT
%       A,B,C,D ==> state space matrices
    

% STABILITY DERIVATIVES
 Y_v = ureal('Y_v', -0.264, 'Percentage', sigma*4.837);
 Y_p = 0;
 L_v = ureal('L_v', -7.349, 'Percentage', sigma*4.927);
 L_p = 0;

% CONTROL DERIVATIVES
 Y_d = ureal('Y_d', 9.568, 'Percentage', sigma*4.647);
 L_d = ureal('L_d', 1079.339, 'Percentage', sigma*2.762);
 
% STATE SPACE MATRICES
A = [Y_v,Y_p,-9.81;L_v,L_p,0;0,1,0];
B = [Y_d;L_d;0];

if nominal
    A = getNominal(A);
    B = getNominal(B);
end


C_p = [0,1,0];
D = 0;
G_p = tf(ss(A,B,C_p,D));

%CLEAN THE NOMINAL MODEL FROM UNPHYSICAL DATA
[num, den] = tfdata(G_p,'v');
tol = 1e-5;
index_n = abs(num) < tol;
index_d = abs(den) < tol;
num(index_n) = 0;
den(index_d) = 0;
G_p = tf(num, den);

end

