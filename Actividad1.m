clear all
close all
clc

tic
syms x(t) y(t) th(t) t %Grados de libertad del robot movil

%Creamos el vector de posicion
xi_inercial = [x; y; th];
disp('Coordenadas generalizadas');
pretty(xi_inercial);

%Creamos el vector de velocidades
xip_inercial = diff(xi_inercial, t);
disp('Velocidades generalizadas');
pretty(xip_inercial);

%define mi vector de posicion y matriz de rotacion
P(:,:,1) = [x;y;th];

R(:,:,1)=[cos(th) -sin(th) 0;
          sin(th) cos(th) 0;
          0 0 1];
%Realizo mi transformacion del marco de referencia global al local....
xi_local = R(:,:,1)* P(:,:,1);

%Defino coordenadas inerciales para un tiempo 1}
x_list = [-5, -3, 5, 0, -6, 10, 9, 5, -1, 6, 5, 7, 11, 20, 10, -9, 1, 3, 15, -10];
y_list = [9, 8, -2, 0, 3, -2, 1, 2, -1, 4, 7, 7, -4, 5, 9, -8, 1, 1, 2, 0];
th_list = [-2, 63, 90, 180, -55, 45, 88, 33, 21, -40, 72, 30, 360, 270, 345, 8, 60, -30, 199, 300];


for i=1:20

    %Defino mi vector de posicion y matriz de rotacion para un tiempo 1
    Pos_i = [x_list(i);y_list(i);th_list(i)];
    
    Rot_i = [cos(th_list(i)) -sin(th_list(i)) 0;
            sin(th_list(i)) cos(th_list(i)) 0;
            0 0 1];

    fprintf('Ejemplo %d: \n',round(i));

    %Realizo mi transformacion del marco de referencia inercial al local...
    disp('Marco de referencia inercial al local;')
    xi_local_i = Rot_i*Pos_i; 
    disp(xi_local_i); 

    %Obtengo la magnitud del vector resultante 
    disp('Magnitud del vector resultante: ')
    magnitud = sqrt(xi_local_i(1)^2 + xi_local_i(2)^2); 
    disp(magnitud); 

    %Compruebo que me devuelve el vector inercial
    disp('Vector inercial: ')
    inv_Rot_i = inv(Rot_i);
    xi_inercial_i = inv_Rot_i*xi_local_i; 
    disp(xi_inercial_i); 

end
