close all;
clc
clear all;

config = readstruct("../sourceFiles/configuration.json");

xBodySize = config.CartBody.xSize;
yBodySize = config.CartBody.ySize;
zBodySize = config.CartBody.zSize;
bodyDensity = config.CartBody.density;

rWheelSize = config.Wheel.radius;
hWheelSize = config.Wheel.width;
wheelDensity = config.Wheel.density;

rPendulumSphere = config.Sphere.radius;
pendulumSphereDensity = config.Sphere.density;

hPendulumBeam = config.Beam.height;

Mb = xBodySize*yBodySize*zBodySize*bodyDensity;
Mw = pi*rWheelSize^2*hWheelSize*wheelDensity;

M = Mb+Mw*4;
m = 4/3*pi*rPendulumSphere^3*pendulumSphereDensity;

g = 9.8;
l = hPendulumBeam+rPendulumSphere;

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


R = 1e-3;
Q = [1,0,0,0;
    0,1,0,0;
    0,0,100,0;
    0,0,0,100];

F = lqr(system, Q, R);

L = ss((A-B*F), B, C, 0);
Ts = 0.001;
sys_d = c2d(system,Ts);

% figure
% step(L);
% figure;
% bodemag(L);
