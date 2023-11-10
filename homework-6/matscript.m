% Cart Table State Space Model
zc = 0.5; %0.724531
g = 9.81;
N_L = 500;
A = [
    0,1,0;
    0,0,1;
    0,0,0
]; 
B = [0;0;1];
C = [1,0,-zc/g];
Ip = 1; % identity matrix of size p
R = 1.0e-6;
Qe = 1.0;
Qx = zeros(3,3);

% Compute ZMP Error State Space Model
B_tilde = [C*B; B];
I_tilde = [Ip; zeros(3,1)];
F_tilde = [C*A; A];
A_tilde = [I_tilde, F_tilde];
Q_tilde = [Qe, zeros(1,3); zeros(3,1), Qx];

% Compute Riccati Equation
K_tilde = dare(A_tilde,B_tilde,Q_tilde,R);

% Compute Preview Control Gains
G_I = inv(R+B_tilde'*K_tilde*B_tilde)*B_tilde'*K_tilde*I_tilde; % integral gain
G_x = inv(R+B_tilde'*K_tilde*B_tilde)*B_tilde'*K_tilde*F_tilde; % state feedback gain
A_c = A_tilde - B_tilde*inv(R+B_tilde'*K_tilde*B_tilde)*B_tilde'*K_tilde*A_tilde;

X = zeros(4, N_L);
X(:,1) = -A_c'*K_tilde*I_tilde;
for l = 2:N_L
    X(:,l) = A_c'*X(:,l-1);
end

G_d = zeros(1, N_L);
G_d(:,1) = -G_I;
for l = 2:N_L
    G_d(:,l) = inv(R+B_tilde'*K_tilde*B_tilde)*B_tilde'*X(:,l-1);
end
