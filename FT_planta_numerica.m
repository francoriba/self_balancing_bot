close all, clear all; clc;
s = tf('s');
g = 9.81;                    % m/s^2 - Aceleración debida a la gravedad
 
%Parametros del Sistema de Rotación
R = 0.035;                % m - radio de la rueda, 35 mm.
r = 0.005;                     % m - radio del rotor, 5mm.
 
m_rueda = 0.0551;            % Kg - masa de la rueda.
m_rot = 0.007;                    % Kg - masa del rotor
m_engranajesYeje = 0.0025;             % Kg - masa de los engranajes de la caja reductora
 
m_w = m_rueda +  m_rot + m_engranajesYeje;
 
J_w = 0.5*m_w*R*R;                     % Kgm2 - inercia de la rueda
b_w = 0;                             % Nms/rad - fricción viscosa de la rueda (despreciada)
 
%Parametros del cuerpo Principal
m_chasis = 0.5584;                      % Kg - masa del chasis del robot (plataformas impresas en 3d + componentes)
m_estat = 0.01;                     % Kg - masa del estator
m_carcasa = 0.0054;                          % Kg - masa de la carcasa del motorreductor
m_b = m_chasis + m_estat + m_carcasa;       % Kg - Masa del cuerpo Principal
 
l = 0.07;                                 
J_b = 0.0024;             % Kgm2 - momento de inercia del chasis refernciado al centro de masa,
%ver el archivo .m llamado "Hallar_Jb_teorema_ejes_paralelos"
                        
 
% Constantes aproximadas de la Planta
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
a22 = C1;
a23 = C1;
a24 = 0;
a31 = 0;
a32 = 0;
a33 = 0;
a34 = 1;
a41 = 0;
a42 = C4;
a43 = C3;
a44 = 0;
A = [a11 a12 a13 a14; a21 a22 a23 a24; a31 a32 a33 a34; a41 a42 a43 a44]
B = [0; C2; 0; C4]
C = [0 0 1 0]

 

[n,m] = ss2tf(A,B,C,0)
step(n,m)
