clear all port;
clc;
clear all;
close all;
fs = 10;


Tiempo=5;


%---LETRA L HORIZONTAL---%
P0 =[]

P1LH = [-30 170 -5]; 
P2LH = [-30 170 -40];
P3LH = [-30 140 -40];
P4LH = [-15 140 -40];
P5LH = [-15 140 -5];


%___P1LH-P2LH___%
Posx1LH = GenPolSV(P1LH(1),P2LH(1),'L',Tiempo,fs);
Posy1LH = GenPolSV(P1LH(2),P2LH(2),'L',Tiempo,fs);
Posz1LH = GenPolSV(P1LH(3),P2LH(3),'L',Tiempo,fs);


% Cinemática Inversa:
ang1LH = zeros(length(Posx1LH),4);
for i = 1:1:length(Posx1LH)
    ang1LH(i,:) = CInversa([Posx1LH(i) Posy1LH(i) Posz1LH(i)],53);
end


% Cinemática Inversa:
ang1LH = zeros(length(Posx1LH),4);
for i = 1:1:length(Posx1LH)
    ang1LH(i,:) = CInversa([Posx1LH(i) Posy1LH(i) Posz1LH(i)],53);
end

%DE AQUÍ AL MOTOR

%___P2LH-P3LH___%
Posx2LH = GenPolSV(P2LH(1),P3LH(1),'L',Tiempo,fs);
Posy2LH = GenPolSV(P2LH(2),P3LH(2),'L',Tiempo,fs);
Posz2LH = GenPolSV(P2LH(3),P3LH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang2LH = zeros(length(Posx2LH),4);
for i = 1:1:length(Posx2LH)
    ang2LH(i,:) = CInversa([Posx2LH(i) Posy2LH(i) Posz2LH(i)],53);
end

%___P3LH-P4LH___%
Posx3LH = GenPolSV(P3LH(1),P4LH(1),'L',Tiempo,fs);
Posy3LH = GenPolSV(P3LH(2),P4LH(2),'L',Tiempo,fs);
Posz3LH = GenPolSV(P3LH(3),P4LH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang3LH = zeros(length(Posx3LH),4);
for i = 1:1:length(Posx3LH)
    ang3LH(i,:) = CInversa([Posx3LH(i) Posy3LH(i) Posz3LH(i)],53);
end

%___P4LH-P5LH___%
Posx4LH = GenPolSV(P4LH(1),P5LH(1),'J',Tiempo,fs);
Posy4LH = GenPolSV(P4LH(2),P5LH(2),'J',Tiempo,fs);
Posz4LH = GenPolSV(P4LH(3),P5LH(3),'J',Tiempo,fs);

% Cinemática Inversa:
ang4LH = zeros(length(Posx4LH),4);
for i = 1:1:length(Posx4LH)
    ang4LH(i,:) = CInversa([Posx4LH(i) Posy4LH(i) Posz4LH(i)],53);
end






angtot = vertcat(ang1LH,ang2LH,ang3LH,ang4LH);

t = 0:0.0001:(length(angtot)-1)*0.0001;
figure(1)
plot(t,angtot(:, 1))
hold on
plot(t,angtot(:, 2))
plot(t,angtot(:, 3))
plot(t,angtot(:, 4))
hold off
legend('\theta1','\theta2','\theta3','\theta4')
grid on 
title('Ángulos de articulaciones para la escritura de L en el plano horizontal')
ylabel('Ángulos(grados)')
xlabel('Tiempo(s)')

ang1mot = angtot(:, 1)+3;
ang2mot = angtot(:, 2)+10;
ang3mot = angtot(:, 3)+100;
ang4mot = angtot(:, 4)+105;

verificacion = vertcat(ang1mot,ang2mot,ang3mot,ang4mot)

for i = 1:1:length(verificacion)
    if verificacion(i) <0|verificacion>180
        error('SINGULARIDAD');
    end
end

MRL = (horzcat(ang1mot,ang2mot,ang3mot,ang4mot))/180;

Tiempo=5;


%---LETRA R HORIZONTAL---%
P0 =[]

P1RH = [-5 140 -5]; 
P2RH = [-5 140 -40];
P3RH = [-5 170 -40];
P4RH = [8 170 -40];
P5RH = [10 168 -40];
P6RH = [10 157 -40];
P7RH = [8 155 -40];
P8RH = [-5 155 -40];
P9RH = [10 140 -40];
P10RH = [10 140 -5];

%___P1RH-P2RH___%
Posx1RH = GenPolSV(P1RH(1),P2RH(1),'L',Tiempo,fs);
Posy1RH = GenPolSV(P1RH(2),P2RH(2),'L',Tiempo,fs);
Posz1RH = GenPolSV(P1RH(3),P2RH(3),'L',Tiempo,fs);


% Cinemática Inversa:
ang1RH = zeros(length(Posx1RH),4);
for i = 1:1:length(Posx1RH)
    ang1RH(i,:) = CInversa([Posx1RH(i) Posy1RH(i) Posz1RH(i)],53);
end


% Cinemática Inversa:
ang1RH = zeros(length(Posx1RH),4);
for i = 1:1:length(Posx1RH)
    ang1RH(i,:) = CInversa([Posx1RH(i) Posy1RH(i) Posz1RH(i)],53);
end

%DE AQUÍ AL MOTOR

%___P2RH-P3RH___%
Posx2RH = GenPolSV(P2RH(1),P3RH(1),'L',Tiempo,fs);
Posy2RH = GenPolSV(P2RH(2),P3RH(2),'L',Tiempo,fs);
Posz2RH = GenPolSV(P2RH(3),P3RH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang2RH = zeros(length(Posx2RH),4);
for i = 1:1:length(Posx2RH)
    ang2RH(i,:) = CInversa([Posx2RH(i) Posy2RH(i) Posz2RH(i)],53);
end

%___P3RH-P4RH___%
Posx3RH = GenPolSV(P3RH(1),P4RH(1),'L',Tiempo,fs);
Posy3RH = GenPolSV(P3RH(2),P4RH(2),'L',Tiempo,fs);
Posz3RH = GenPolSV(P3RH(3),P4RH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang3RH = zeros(length(Posx3RH),4);
for i = 1:1:length(Posx3RH)
    ang3RH(i,:) = CInversa([Posx3RH(i) Posy3RH(i) Posz3RH(i)],53);
end

%___P4RH-P5RH___%
Posx4RH = GenPolSV(P4RH(1),P5RH(1),'L',Tiempo,fs);
Posy4RH = GenPolSV(P4RH(2),P5RH(2),'L',Tiempo,fs);
Posz4RH = GenPolSV(P4RH(3),P5RH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang4RH = zeros(length(Posx4RH),4);
for i = 1:1:length(Posx4RH)
    ang4RH(i,:) = CInversa([Posx4RH(i) Posy4RH(i) Posz4RH(i)],53);
end

%___P5RH-P6RH___%
Posx5RH = GenPolSV(P5RH(1),P6RH(1),'L',Tiempo,fs);
Posy5RH = GenPolSV(P5RH(2),P6RH(2),'L',Tiempo,fs);
Posz5RH = GenPolSV(P5RH(3),P6RH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang5RH = zeros(length(Posx5RH),4);
for i = 1:1:length(Posx5RH)
    ang5RH(i,:) = CInversa([Posx5RH(i) Posy5RH(i) Posz5RH(i)],53);
end

%___P6RH-P7RH___%
Posx6RH = GenPolSV(P6RH(1),P7RH(1),'L',Tiempo,fs);
Posy6RH = GenPolSV(P6RH(2),P7RH(2),'L',Tiempo,fs);
Posz6RH = GenPolSV(P6RH(3),P7RH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang6RH = zeros(length(Posx6RH),4);
for i = 1:1:length(Posx6RH)
    ang6RH(i,:) = CInversa([Posx6RH(i) Posy6RH(i) Posz6RH(i)],53);
end

%___P7RH-P8RH___%
Posx7RH = GenPolSV(P7RH(1),P8RH(1),'L',Tiempo,fs);
Posy7RH = GenPolSV(P7RH(2),P8RH(2),'L',Tiempo,fs);
Posz7RH = GenPolSV(P7RH(3),P8RH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang7RH = zeros(length(Posx7RH),4);
for i = 1:1:length(Posx7RH)
    ang7RH(i,:) = CInversa([Posx7RH(i) Posy7RH(i) Posz7RH(i)],53);
end

%___P8RH-P9RH___%
Posx8RH = GenPolSV(P8RH(1),P9RH(1),'L',Tiempo,fs);
Posy8RH = GenPolSV(P8RH(2),P9RH(2),'L',Tiempo,fs);
Posz8RH = GenPolSV(P8RH(3),P9RH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang8RH = zeros(length(Posx8RH),4);
for i = 1:1:length(Posx8RH)
    ang8RH(i,:) = CInversa([Posx8RH(i) Posy8RH(i) Posz8RH(i)],53);
end

%___P9RH-P10RH___%
Posx9RH = GenPolSV(P9RH(1),P10RH(1),'L',Tiempo,fs);
Posy9RH = GenPolSV(P9RH(2),P10RH(2),'L',Tiempo,fs);
Posz9RH = GenPolSV(P9RH(3),P10RH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang9RH = zeros(length(Posx9RH),4);
for i = 1:1:length(Posx9RH)
    ang9RH(i,:) = CInversa([Posx9RH(i) Posy9RH(i) Posz9RH(i)],53);
end



angtot = vertcat(ang1RH,ang2RH,ang3RH,ang4RH,ang5RH,ang6RH,ang7RH,ang8RH,ang9RH);

t = 0:0.0001:(length(angtot)-1)*0.0001;
figure(2)
plot(t,angtot(:, 1))
hold on
plot(t,angtot(:, 2))
plot(t,angtot(:, 3))
plot(t,angtot(:, 4))
hold off
legend('\theta1','\theta2','\theta3','\theta4')
grid on 
title('Ángulos de articulaciones para la escritura de R en el plano horizontal')
ylabel('Ángulos(grados)')
xlabel('Tiempo(s)')


ang1mot = angtot(:, 1)+3;
ang2mot = angtot(:, 2)+10;
ang3mot = angtot(:, 3)+100;
ang4mot = angtot(:, 4)+105;

verificacion = vertcat(ang1mot,ang2mot,ang3mot,ang4mot)

for i = 1:1:length(verificacion)
    if verificacion(i) <0|verificacion>180
        error('SINGULARIDAD');
    end
end

MRR = (horzcat(ang1mot,ang2mot,ang3mot,ang4mot))/180;

Tiempo=5;


%---LETRA A HORIZONTAL---%
P0 =[]

P1MH = [20 140 -5]; 
P2MH = [20 140 -40];
P3MH = [20 170 -40];
P4MH = [30 140 -40];
P5MH = [40 170 -40];
P6MH = [40 140 -40];
P7MH = [40 140 -5];


%___P1MH-P2MH___%
Posx1MH = GenPolSV(P1MH(1),P2MH(1),'L',Tiempo,fs);
Posy1MH = GenPolSV(P1MH(2),P2MH(2),'L',Tiempo,fs);
Posz1MH = GenPolSV(P1MH(3),P2MH(3),'L',Tiempo,fs);


% Cinemática Inversa:
ang1MH = zeros(length(Posx1MH),4);
for i = 1:1:length(Posx1MH)
    ang1MH(i,:) = CInversa([Posx1MH(i) Posy1MH(i) Posz1MH(i)],53);
end


% Cinemática Inversa:
ang1MH = zeros(length(Posx1MH),4);
for i = 1:1:length(Posx1MH)
    ang1MH(i,:) = CInversa([Posx1MH(i) Posy1MH(i) Posz1MH(i)],53);
end

%DE AQUÍ AL MOTOR

%___P2MH-P3MH___%
Posx2MH = GenPolSV(P2MH(1),P3MH(1),'L',Tiempo,fs);
Posy2MH = GenPolSV(P2MH(2),P3MH(2),'L',Tiempo,fs);
Posz2MH = GenPolSV(P2MH(3),P3MH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang2MH = zeros(length(Posx2MH),4);
for i = 1:1:length(Posx2MH)
    ang2MH(i,:) = CInversa([Posx2MH(i) Posy2MH(i) Posz2MH(i)],53);
end

%___P3MH-P4MH___%
Posx3MH = GenPolSV(P3MH(1),P4MH(1),'L',Tiempo,fs);
Posy3MH = GenPolSV(P3MH(2),P4MH(2),'L',Tiempo,fs);
Posz3MH = GenPolSV(P3MH(3),P4MH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang3MH = zeros(length(Posx3MH),4);
for i = 1:1:length(Posx3MH)
    ang3MH(i,:) = CInversa([Posx3MH(i) Posy3MH(i) Posz3MH(i)],53);
end

%___P4MH-P5MH___%
Posx4MH = GenPolSV(P4MH(1),P5MH(1),'L',Tiempo,fs);
Posy4MH = GenPolSV(P4MH(2),P5MH(2),'L',Tiempo,fs);
Posz4MH = GenPolSV(P4MH(3),P5MH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang4MH = zeros(length(Posx4MH),4);
for i = 1:1:length(Posx4MH)
    ang4MH(i,:) = CInversa([Posx4MH(i) Posy4MH(i) Posz4MH(i)],53);
end

%___P5MH-P6MH___%
Posx5MH = GenPolSV(P5MH(1),P6MH(1),'L',Tiempo,fs);
Posy5MH = GenPolSV(P5MH(2),P6MH(2),'L',Tiempo,fs);
Posz5MH = GenPolSV(P5MH(3),P6MH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang5MH = zeros(length(Posx5MH),4);
for i = 1:1:length(Posx5MH)
    ang5MH(i,:) = CInversa([Posx5MH(i) Posy5MH(i) Posz5MH(i)],53);
end

%___P6MH-P7MH___%
Posx6MH = GenPolSV(P6MH(1),P7MH(1),'J',Tiempo,fs);
Posy6MH = GenPolSV(P6MH(2),P7MH(2),'J',Tiempo,fs);
Posz6MH = GenPolSV(P6MH(3),P7MH(3),'J',Tiempo,fs);

% Cinemática Inversa:
ang6MH = zeros(length(Posx6MH),4);
for i = 1:1:length(Posx6MH)
    ang6MH(i,:) = CInversa([Posx6MH(i) Posy6MH(i) Posz6MH(i)],53);
end






angtot = vertcat(ang1MH,ang2MH,ang3MH,ang4MH,ang5MH,ang6MH);

t = 0:0.0001:(length(angtot)-1)*0.0001;
figure(3)
plot(t,angtot(:, 1))
hold on
plot(t,angtot(:, 2))
plot(t,angtot(:, 3))
plot(t,angtot(:, 4))
hold off
legend('\theta1','\theta2','\theta3','\theta4')
grid on 
title('Ángulos de articulaciones para la escritura de M en el plano horizontal')
ylabel('Ángulos(grados)')
xlabel('Tiempo(s)')

ang1mot = angtot(:, 1)+3;
ang2mot = angtot(:, 2)+10;
ang3mot = angtot(:, 3)+100;
ang4mot = angtot(:, 4)+105;

verificacion = vertcat(ang1mot,ang2mot,ang3mot,ang4mot)

for i = 1:1:length(verificacion)
    if verificacion(i) <0|verificacion>180
        error('SINGULARIDAD');
    end
end

MRM = (horzcat(ang1mot,ang2mot,ang3mot,ang4mot))/180;


M_TotH2 = vertcat(MRL,MRR,MRM);

a = arduino('COM5','Mega2560');

ServoBase = servo(a,'D2'); % Pin Digital / Cita 1
ServoHombro = servo(a,'D6'); % Pin Digital / Cita 2
ServoHombroInv = servo(a,'D4'); % Pin Digital / Cita 2 Invertido
ServoCodo = servo(a,'D8'); % Pin Digital / Cita 3
ServoMuneca = servo(a,'D10'); % Pin Digital / Cita 4

[numFilas , numColumas] = size(M_TotH2);

for i=1:numFilas
    writePosition( ServoBase  , M_TotH2(i,1) ); % Angulo Motor Cita 1
    writePosition( ServoHombro, M_TotH2(i,2) ); % Angulo Motor Cita 2
    writePosition( ServoHombroInv, 1-M_TotH2(i,2) ); % Angulo Motor 2 Invertido
    writePosition( ServoCodo  , M_TotH2(i,3) ); % Angulo Motor Cita 3
    writePosition( ServoMuneca, M_TotH2(i,4) ); % Angulo Motor Cita 4
    pause(0.0001);
end
