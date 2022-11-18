m = 1;
M = 4;
L = 2;
g = -10;
d  = 1;

s = -1; %switch for peudulum up (s=1)
A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -s*d/(M*L) -s* (m+M)*g/(M*L) 0];

B = [0; 1/M; 0; s*1/(M*L)];

C = [ 1; 0; 0; 0].';

D = zeros(size(C,1),size(B,2));

Mscale = [eye(4); zeros(2,4)];
Mscale1 = [eye(3), eye(3)];
Mscale2 = [eye(2), eye(2)];
%% Initial conditions
x0 = [1.5 0 pi 0];

%% Augmented system with noise and disturbance
Vd = 0.1*eye(4); % disturbance covariance
Vn = 1; %noise covariance

BF = [ B Vd 0*B]; %augmented inputs with disturbance and noise

sysC = ss(A,BF,C,[0 0 0 0 0 Vn]); %build state space system

sysFullOutput = ss( A, sysC.B, eye(4),zeros(4,size(BF,2))); % system with full state output, disturbance, no noise

%% Building LQR controller
Q=diag([1; 1; 10; 100]);
R = 0.1;
Kr = lqr(sysC.A,sysC.B,Q,R);

%% discreat time LQR controller
sysC_dis = c2d(sysC,0.01);
Kr_dis = lqr(sysC_dis.A,sysC_dis.B,Q,R);

%% Arbitary controller
des_pol = [ -1; -1 ; -1; -1];
Ka = place(A,sysC.B,des_pol);
%% Build Kalman Filter
[Kf,P,E] = lqe(A,Vd,C,Vd,Vn);
% Kf = lqr(A',C',Vd,Vn)';

sysKf = ss(A-Kf*C, [B Kf], eye(4), 0*[B Kf]);

%% Estimat  linearized system in "down" position (Gantry crane)
dt = 0.01;
t = 0:dt:50;

uDist = rand(4,size(t,2));
uNoise = rand(size(t));
u = 0*t;
u(100:120) = 100; %impulse
u(1500:1520) = -100; %impulse

uAug =[u; Vd*Vd*uDist; uNoise];

[y,t] = lsim(sysC, uAug, t);
figure(1)
plot(t,y);

%% plot of true system
[xtrue,t] = lsim(sysFullOutput,uAug,t);
hold on;
plot(t,xtrue(:,1),'r','Linewidth',2.0);

%% plot of the estimations from the Kalman Filter
[x_hat,t]=lsim(sysKf,[u;y'],t);
hold on;
plot(t,x_hat(:,1),'k--','Linewidth',2.0);

%%
hold off;
figure(2)
plot(t,xtrue,t,x_hat);

