close all;
clc
clear all;

xBodySize = 2;
yBodySize = 0.5;
zBodySize = 1;
bodyDensity = 30000;

rWheelSize = 0.5;
hWheelSize = 0.2;
wheelDensity = 15000;

rPendulumSphere = 0.2;
pendulumSphereDensity = 3000;

hPendulumBeam = 4;

Mb = xBodySize*yBodySize*zBodySize*bodyDensity;
Mw = pi*rWheelSize^2*hWheelSize*wheelDensity;

M = Mb+Mw*4;
m = 4/3*pi*rPendulumSphere^3*pendulumSphereDensity;

g = 9.8;
l = hPendulumBeam+rPendulumSphere/2;

A = [0, 1, 0, 0
    (M+m)*g/(M*l), 0, 0, 0
    0, 0, 0, 1
    -m*g/M, 0, 0, 0];

B = [0
    -1/(M*l)
    0
    1/M];

C = [1, 0, 0, 0
    0, 1, 0, 0
    0, 0, 1, 0
    0, 0, 0, 1
    ];

D = 0;

system = ss(A,B,C,D);


R = 1e-8;
Q = [100,0,0,0;
    0,100,0,0;
    0,0,10,0;
    0,0,0,10];

F = lqr(system, Q, R);

L = ss((A-B*F), B, C, 0);
Ts = 0.001;
sys_d = c2d(system,Ts);

% figure
% step(L);
% figure;
% bodemag(L);
