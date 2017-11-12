'''
%% Clear Workspace
clear, clc
%% Discrete State Space Model
C = [1 0];
D  = 0;
%Ad = [0.999999999999999,20.0000000000003;5.51209569895417e-07,0.361661728228695];
%Bd = [-2.91801628164788e-16;0.000559213179912614];
Ad = [0.999999506241886,0.0202753749834943;-0.000429507455155225,0.436436058875644];
Bd = [0.000153624030679276;0.457408053609279];
Ts = .02 % 20 ms sample time

Kd = [24.4454174931646 0.815490685544169] % LQR gain for state feedback
%% Steady State error

s = ss(Ad-Bd*Kd,Bd,C,D,Ts);
SSerror = abs(1-dcgain(s))
%% Integral Control Design

% Integral State Space Model

Cext = [C 0];
Aext = [Ad [0; 0]; -C 0];
Bext = [Bd; 0];

%% LQR for Integral Gains
N = 10;
Kidarray = [];
Qtheta = logspace(-1,log10(10),N);
Qomega = Qtheta;
Qe = Qtheta;
goodindx = zeros(0,2);
count = 1;
for i = 1:N
    for j = 1:N
        for k = 1:N
        Q = diag([Qtheta(i) Qomega(j) Qe(k)]);
        R = 1;
        [Kd,~,lambda] = dlqr(Aext,Bext,Q,R);
        KIdarray(:,count) = Kd;
        count = count + 1;
        end
    end
end
'''