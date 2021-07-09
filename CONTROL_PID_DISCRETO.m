clc,clear all,close all;
To=0.1;
tfin=1000;
t=[0:To:tfin];
retardo=15; %% si hacen con mas retardo deja de funcionar el pid pilas con eso
for p=1:1:length(t)
 t1(p)=heaviside(t(p)-20);
end
%Consideraciones iniciales de los tanques 
h1(1)=0;
%Parametros de los tanques 1 y 2 
k1=0.05;
k2=0.015;
A=0.5;
g=9.8;
%Puntos de equilibrio de los tanques
a1_equi=0.6; %aperturas de las valvulas entre 0-1
a2_equi=0.4;
%aperuras de las valvulas para los tanques SEÑAL PASO DE VARIOS NIVELES 

%
a1=[0.6*ones(1,2000) 0.55*ones(1,2000) 0.6*ones(1,2000) 0.70*ones(1,2000)...
0.6*ones(1,2001)];%%%%%% dividir tiempo total para periodo de muestreo
a2=0.4*ones(1,length(t));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                           %%%MODELAMIENTO%%%%%%%
y(1)=0;
y_1(1)=0;
y_2(1)=0;
u(1)=0;
u_1(1)=0;
u_2(1)=0;
%% controlador
sp=1*ones(1,length(t));
%% contantes del controlador
kp=0.67572707393869;
ki=0.033451562378585;
kd=0.0672381917700831;
%% senales de control
control(1)=0;
control_1(1)=0;
%% errores del sistema
e(1)=0;
e_1(1)=0;
e_2(1)=0;
for k=1:length(t)
e(k)=0.2*tanh((1/0.2)*(sp(k)-h1(k)));
control(k)=control_1(k)+kp*(e(k)-e_1(k))+ki*To*e_1(k)+(kd/To)*(e(k)-2*e_1(k)+e_2(k));

if k<=retardo/To
    a1retardo(k)=0;
    else 
    a1retardo(k)=control(k-retardo/To);
    end
%a1retardo(k)=control(k);
%% sistema linealizado en ecuacion en diferencias en base a esta se diseno el pid 
u(k)=a1retardo(k);    
y(k+1)=0.04996*y_1(k)+0.9394*y_2(k)+0.02232*u_1(k);

%%modelo tanque no lineal 
hp1=((k1*(a1retardo(k)*t1(k))-k2*a2(k)*sqrt(2*g*h1(k)))/A);
h1(k+1)=hp1*To+h1(k);

%% actaualizacion valores para el controlador en dominio de la frecuancia discreta
control_1(k+1)=control(k);
e_2(k+1)=e_1(k);
e_1(k+1)=e(k);
%% actaulizacion de datos del sistema en dominio de la frecuncia discreta
u_1(k+1)=u(k);

y_1(k+1)=y(k+1);
y_2(k+1)=y_1(k+1);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%ANIMACION%%%%%%%%%
figure;
for i=1:20:length(t)
H1= plot(t(1:i),h1(1:i),'r','LineWidth',2);hold on ;
H2= plot(t(1:i),control(1:i),'b','LineWidth',2);hold on ;
H3= plot(t(1:i),sp(1:i),'g','LineWidth',2);hold on ;
H4= plot(t(1:i),a1retardo(1:i),'--b','LineWidth',2);hold on ;
H5= plot(t(1:i),y(1:i),'--r','LineWidth',2);hold on ;
grid on ;
drawnow;
hold off;
end
legend('Modelo no linealizado tanque con retardo','control value','Sp','control retardo');
h1=h1(1,1:length(t));
