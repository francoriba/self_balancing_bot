close all, clear all; clc;
s = tf('s');
syms s C1 C2 C3 C4 Cm1 Cm2 Cm3

%Matriz A del sistema 
a11 = 0;
a12 = 1;
a13 = 0;
a14 = 0;
a21 = 0;
a22 = -C2*Cm2;
a23 = C1;
a24 = C2*Cm3;
a31 = 0;
a32 = 0;
a33 = 0;
a34 = 1;
a41 = 0;
a42 = -C4*Cm2;
a43 = C3;
a44 = C4*Cm3;
A = [a11 a12 a13 a14; a21 a22 a23 a24; a31 a32 a33 a34; a41 a42 a43 a44]
B = [0; C2*Cm1; 0; C4*Cm1]
C = [0 0 1 0]

%MODELO GENERAL
I = s*eye(4,4)
x = I-A;
i=inv(x);
r1 = C*i*B

pretty(collect(r1,'s'));


