close all, clear all; clc;
s = tf('s');
gLa = 1.79*s/(s^3+0.008917*s^2-141.3*s+0.0452);

glc= minreal(gLa/(1+gLa))