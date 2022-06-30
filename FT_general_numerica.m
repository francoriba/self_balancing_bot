close all, clear all; clc;
s = tf('s');

g = 9.81;                                          % m/s^2 - aceleración de la gravedad
%Parametros del prototipo
% --- Parametros de los motorreductores (del datasheet) 

Tstall = 0.011;                                      % stall torque en Kg/m -> 1.1 Kg cm = 0.1078 Nm = 0.011 Kg m
RPMnoload = 230;                                     % No-load RPM con Vin = 6V.
wnl = (RPMnoload*2*pi)/60;                           % velocidad de rotación del eje de salida de la caja reductora sin carga en rad/seg. 
Inl = 0.06;                                          % No load current into motor, from measurement.
Vin = 6;                                             % Volt. tensión de referencia durante el ensayo
Istall = 1.5;                                        % Amp, corriente stall de la armadura
Ng = 48;                                             % Gear ratio o relación de reducción
Dg = 0;                                              % kgm/rad/sec - perdida viscosa de los engranajes
La = 0;                                              % H - Asumimos inductancia de la armadura despreciable
Jm = 0.0000175;                                      %calculado en base a la masa y el radio del rotor considerado como cilindro sólido (ver anexo de mediciones)

Ra = Vin/Istall;                                     % Ohm - Resistencia de la armadura.
Kt = (Tstall/(Vin*Ng))*Ra;                           % Nm/A -  Constante de torque del motor
Kb = (Vin-Ra*Inl)/(wnl*Ng);                          % V seg/rad - Constante de contra fem 

% --- Parametros del controlador de motores (L298N)---
Kmd = 1.0;                                           % Ganancia de tensión del L298N (depende de la implementación)

% --- Parametros del sistema de rotación ---
R = 0.035;                                           % m - radio de la rueda, 35 mm.
r = 0.005;                                           % m - radio del rotor, 5mm.

m_rueda = 0.0551;                                    % Kg - masa de la rueda.
m_rot = 0.007;                                       % Kg - masa del rotor
m_engranajesYeje = 0.0025;                           % Kg - masa de los engranajes de la caja reductora

m_w = m_rueda;

J_w = 0.5*m_rueda*R*R;                               % Kgm2 - inercia de la rueda
b_w = 0;                                             % Nms/rad - fricción viscosa de la rueda (despreciada)

% --- Parametros del cuerpo Principal ---
m_chasis = 0.5584;                                   % Kg - masa del chasis del robot (plataformas impresas en 3d + componentes)
m_estat = 0.01;                                      % Kg - masa del estator
m_carcasa = 0.0054;                                  % Kg - masa de la carcasa del motorreductor
m_b = m_chasis + m_estat + m_carcasa+m_rot+m_engranajesYeje;                % Kg - Masa del cuerpo Principal

              
l = 0.07;                                           % m - distancia desde el centro de masa del chasis al eje de la rueda
J_b = 0.0024;                   % Kgm2 - momento de inercia del chasis refernciado al centro de masa, ver el archivo .m llamado "Hallar_Jb_teorema_ejes_paralelos"
                        
%Constantes aproximadas del motor
Cm1 = (Ng*Kt)/Ra;
Cm2 = (Kb*Kt*Ng)/(Ra*R);
Cm3 = Kb*Kt*(Ng^2)/Ra;  

%Constantes aproximadas de la Planta (con dos ruedas)
M = m_b + 2*(J_w/(R^2) + m_w)-(((m_b*l))^2/((J_b+m_b*l^2 )));
J = (J_b+m_b*l^2 )-(m_b*l)^2/[m_b+2 *(J_w * 1/R^2 +m_w  )]; 
C1 = -1/M*((m_b*l)^2*g)/((J_b+m_b*l^2 ))
C2 = 1/M*2*[(m_b*l)/((J_b+m_b * l^2 ) )+1/R]
C3 = 1/J*(m_b*g*l)
C4 = -1/J*2/R*[(m_b*l)/(m_b+2*(J_w/R^2+m_w))+R]

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



[a,b]=ss2tf(A,B,C,0)
G = minreal(tf(a,b))
pole(G)

figure();
impulse(G/10);grid;
set(gca,'XMinorTick','on','YMinorTick','on') % Para tener subdivisiones entre los valores!
title({'Open-Loop Impulse Disturbance Response'});
xlabel('time');
ylabel('\theta_{i} (rad)');

figure();
rlocus (G);grid;
set(gca,'XMinorTick','on','YMinorTick','on') % Para tener subdivisiones entre los valores!
title({'Lugar de Raices de la Planta'});
xlabel('Eje imaginario');
ylabel('Eje real');