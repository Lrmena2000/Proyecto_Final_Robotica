clear all port;
clc;
clear all;
close all;
fs = 10;


Tiempo=5;


%---LETRA A HORIZONTAL---%
P0 =[]

P1AH = [-20 140 -5]; 
P2AH = [-20 140 -42];
P3AH = [-12.5 170 -42];
P4AH = [-5 140 -37];
P5AH = [-5 140 -5];
P6AH = [-16.25 155 -5];
P7AH = [-16.25 155 -40];
P8AH = [-8.75 155 -40];
P9AH = [-8.75 155 -5];

%___P1AH-P2AH___%
Posx1AH = GenPolSV(P1AH(1),P2AH(1),'L',Tiempo,fs);
Posy1AH = GenPolSV(P1AH(2),P2AH(2),'L',Tiempo,fs);
Posz1AH = GenPolSV(P1AH(3),P2AH(3),'L',Tiempo,fs);


% Cinemática Inversa:
ang1AH = zeros(length(Posx1AH),4);
for i = 1:1:length(Posx1AH)
    ang1AH(i,:) = CInversa([Posx1AH(i) Posy1AH(i) Posz1AH(i)],53);
end


% Cinemática Inversa:
ang1AH = zeros(length(Posx1AH),4);
for i = 1:1:length(Posx1AH)
    ang1AH(i,:) = CInversa([Posx1AH(i) Posy1AH(i) Posz1AH(i)],53);
end

%DE AQUÍ AL MOTOR

%___P2AH-P3AH___%
Posx2AH = GenPolSV(P2AH(1),P3AH(1),'L',Tiempo,fs);
Posy2AH = GenPolSV(P2AH(2),P3AH(2),'L',Tiempo,fs);
Posz2AH = GenPolSV(P2AH(3),P3AH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang2AH = zeros(length(Posx2AH),4);
for i = 1:1:length(Posx2AH)
    ang2AH(i,:) = CInversa([Posx2AH(i) Posy2AH(i) Posz2AH(i)],53);
end

%___P3AH-P4AH___%
Posx3AH = GenPolSV(P3AH(1),P4AH(1),'L',Tiempo,fs);
Posy3AH = GenPolSV(P3AH(2),P4AH(2),'L',Tiempo,fs);
Posz3AH = GenPolSV(P3AH(3),P4AH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang3AH = zeros(length(Posx3AH),4);
for i = 1:1:length(Posx3AH)
    ang3AH(i,:) = CInversa([Posx3AH(i) Posy3AH(i) Posz3AH(i)],53);
end

%___P4AH-P5AH___%
Posx4AH = GenPolSV(P4AH(1),P5AH(1),'L',Tiempo,fs);
Posy4AH = GenPolSV(P4AH(2),P5AH(2),'L',Tiempo,fs);
Posz4AH = GenPolSV(P4AH(3),P5AH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang4AH = zeros(length(Posx4AH),4);
for i = 1:1:length(Posx4AH)
    ang4AH(i,:) = CInversa([Posx4AH(i) Posy4AH(i) Posz4AH(i)],53);
end

%___P5AH-P6AH___%
Posx5AH = GenPolSV(P5AH(1),P6AH(1),'J',Tiempo,fs);
Posy5AH = GenPolSV(P5AH(2),P6AH(2),'J',Tiempo,fs);
Posz5AH = GenPolSV(P5AH(3),P6AH(3),'J',Tiempo,fs);

% Cinemática Inversa:
ang5AH = zeros(length(Posx5AH),4);
for i = 1:1:length(Posx5AH)
    ang5AH(i,:) = CInversa([Posx5AH(i) Posy5AH(i) Posz5AH(i)],53);
end

%___P6AH-P7AH___%
Posx6AH = GenPolSV(P6AH(1),P7AH(1),'L',Tiempo,fs);
Posy6AH = GenPolSV(P6AH(2),P7AH(2),'L',Tiempo,fs);
Posz6AH = GenPolSV(P6AH(3),P7AH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang6AH = zeros(length(Posx6AH),4);
for i = 1:1:length(Posx6AH)
    ang6AH(i,:) = CInversa([Posx6AH(i) Posy6AH(i) Posz6AH(i)],53);
end

%___P7AH-P8AH___%
Posx7AH = GenPolSV(P7AH(1),P8AH(1),'L',Tiempo,fs);
Posy7AH = GenPolSV(P7AH(2),P8AH(2),'L',Tiempo,fs);
Posz7AH = GenPolSV(P7AH(3),P8AH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang7AH = zeros(length(Posx7AH),4);
for i = 1:1:length(Posx7AH)
    ang7AH(i,:) = CInversa([Posx7AH(i) Posy7AH(i) Posz7AH(i)],53);
end

%___P8AH-P9AH___%
Posx8AH = GenPolSV(P8AH(1),P9AH(1),'L',Tiempo,fs);
Posy8AH = GenPolSV(P8AH(2),P9AH(2),'L',Tiempo,fs);
Posz8AH = GenPolSV(P8AH(3),P9AH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang8AH = zeros(length(Posx8AH),4);
for i = 1:1:length(Posx8AH)
    ang8AH(i,:) = CInversa([Posx8AH(i) Posy8AH(i) Posz8AH(i)],53);
end




angtot = vertcat(ang1AH,ang2AH,ang3AH,ang4AH,ang5AH,ang6AH,ang7AH,ang8AH);


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
title('Ángulos de articulaciones para la escritura de A en el plano horizontal')
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

MRA = (horzcat(ang1mot,ang2mot,ang3mot,ang4mot))/180;


Tiempo=5;


%---LETRA M HORIZONTAL---%
P0 =[]

P1MH = [10 140 -5]; 
P2MH = [10 140 -42];
P3MH = [10 170 -40];
P4MH = [25 140 -40];
P5MH = [40 170 -42];
P6MH = [40 140 -41];
P7AMH = [40 140 -5];


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
Posx6MH = GenPolSV(P6MH(1),P7AMH(1),'J',Tiempo,fs);
Posy6MH = GenPolSV(P6MH(2),P7AMH(2),'J',Tiempo,fs);
Posz6MH = GenPolSV(P6MH(3),P7AMH(3),'J',Tiempo,fs);

% Cinemática Inversa:
ang6MH = zeros(length(Posx6MH),4);
for i = 1:1:length(Posx6MH)
    ang6MH(i,:) = CInversa([Posx6MH(i) Posy6MH(i) Posz6MH(i)],53);
end






angtot = vertcat(ang1MH,ang2MH,ang3MH,ang4MH,ang5MH,ang6MH);

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





Tiempo=5;


%---LETRA B HORIZONTAL---%
P0 =[]

P1BH = [50 140 -5]; 
P2BH = [50 140 -43];
P3BH = [50 160 -43];
P4BH = [65 160 -44];
P5BH = [67 158 -42];
P6BH = [67 152 -42];
P7BH = [65 150 -42];
P8BH = [50 150 -42];
P9BH = [50 150 -5];
P10BH = [65 150 -5];
P11BH = [65 150 -42];
P12BH = [69 148 -42];
P13BH = [69 142 -42];
P14BH = [65 140 -41];
P15BH = [50 140 -41];
P16BH = [50 140 -5];


%___P1BH-P2BH___%
Posx1BH = GenPolSV(P1BH(1),P2BH(1),'L',Tiempo,fs);
Posy1BH = GenPolSV(P1BH(2),P2BH(2),'L',Tiempo,fs);
Posz1BH = GenPolSV(P1BH(3),P2BH(3),'L',Tiempo,fs);


% Cinemática Inversa:
ang1BH = zeros(length(Posx1BH),4);
for i = 1:1:length(Posx1BH)
    ang1BH(i,:) = CInversa([Posx1BH(i) Posy1BH(i) Posz1BH(i)],53);
end


% Cinemática Inversa:
ang1BH = zeros(length(Posx1BH),4);
for i = 1:1:length(Posx1BH)
    ang1BH(i,:) = CInversa([Posx1BH(i) Posy1BH(i) Posz1BH(i)],53);
end

%DE AQUÍ AL MOTOR

%___P2BH-P3BH___%
Posx2BH = GenPolSV(P2BH(1),P3BH(1),'L',Tiempo,fs);
Posy2BH = GenPolSV(P2BH(2),P3BH(2),'L',Tiempo,fs);
Posz2BH = GenPolSV(P2BH(3),P3BH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang2BH = zeros(length(Posx2BH),4);
for i = 1:1:length(Posx2BH)
    ang2BH(i,:) = CInversa([Posx2BH(i) Posy2BH(i) Posz2BH(i)],53);
end

%___P3BH-P4BH___%
Posx3BH = GenPolSV(P3BH(1),P4BH(1),'L',Tiempo,fs);
Posy3BH = GenPolSV(P3BH(2),P4BH(2),'L',Tiempo,fs);
Posz3BH = GenPolSV(P3BH(3),P4BH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang3BH = zeros(length(Posx3BH),4);
for i = 1:1:length(Posx3BH)
    ang3BH(i,:) = CInversa([Posx3BH(i) Posy3BH(i) Posz3BH(i)],53);
end

%___P4BH-P5BH___%
Posx4BH = GenPolSV(P4BH(1),P5BH(1),'L',Tiempo,fs);
Posy4BH = GenPolSV(P4BH(2),P5BH(2),'L',Tiempo,fs);
Posz4BH = GenPolSV(P4BH(3),P5BH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang4BH = zeros(length(Posx4BH),4);
for i = 1:1:length(Posx4BH)
    ang4BH(i,:) = CInversa([Posx4BH(i) Posy4BH(i) Posz4BH(i)],53);
end

%___P5BH-P6BH___%
Posx5BH = GenPolSV(P5BH(1),P6BH(1),'L',Tiempo,fs);
Posy5BH = GenPolSV(P5BH(2),P6BH(2),'L',Tiempo,fs);
Posz5BH = GenPolSV(P5BH(3),P6BH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang5BH = zeros(length(Posx5BH),4);
for i = 1:1:length(Posx5BH)
    ang5BH(i,:) = CInversa([Posx5BH(i) Posy5BH(i) Posz5BH(i)],53);
end

%___P6BH-P7BH___%
Posx6BH = GenPolSV(P6BH(1),P7BH(1),'L',Tiempo,fs);
Posy6BH = GenPolSV(P6BH(2),P7BH(2),'L',Tiempo,fs);
Posz6BH = GenPolSV(P6BH(3),P7BH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang6BH = zeros(length(Posx6BH),4);
for i = 1:1:length(Posx6BH)
    ang6BH(i,:) = CInversa([Posx6BH(i) Posy6BH(i) Posz6BH(i)],53);
end

%___P7BH-P8BH___%
Posx7BH = GenPolSV(P7BH(1),P8BH(1),'L',Tiempo,fs);
Posy7BH = GenPolSV(P7BH(2),P8BH(2),'L',Tiempo,fs);
Posz7BH = GenPolSV(P7BH(3),P8BH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang7BH = zeros(length(Posx7BH),4);
for i = 1:1:length(Posx7BH)
    ang7BH(i,:) = CInversa([Posx7BH(i) Posy7BH(i) Posz7BH(i)],53);
end

%___P8BH-P9BH___%
Posx8BH = GenPolSV(P8BH(1),P9BH(1),'L',Tiempo,fs);
Posy8BH = GenPolSV(P8BH(2),P9BH(2),'L',Tiempo,fs);
Posz8BH = GenPolSV(P8BH(3),P9BH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang8BH = zeros(length(Posx8BH),4);
for i = 1:1:length(Posx8BH)
    ang8BH(i,:) = CInversa([Posx8BH(i) Posy8BH(i) Posz8BH(i)],53);
end

%___P9BH-P10BH___%
Posx9BH = GenPolSV(P9BH(1),P10BH(1),'L',Tiempo,fs);
Posy9BH = GenPolSV(P9BH(2),P10BH(2),'L',Tiempo,fs);
Posz9BH = GenPolSV(P9BH(3),P10BH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang9BH = zeros(length(Posx9BH),4);
for i = 1:1:length(Posx9BH)
    ang9BH(i,:) = CInversa([Posx9BH(i) Posy9BH(i) Posz9BH(i)],53);
end

%___P10BH-P11BH___%
Posx10BH = GenPolSV(P10BH(1),P11BH(1),'L',Tiempo,fs);
Posy10BH = GenPolSV(P10BH(2),P11BH(2),'L',Tiempo,fs);
Posz10BH = GenPolSV(P10BH(3),P11BH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang10BH = zeros(length(Posx10BH),4);
for i = 1:1:length(Posx10BH)
    ang10BH(i,:) = CInversa([Posx10BH(i) Posy10BH(i) Posz10BH(i)],53);
end

%___P11BH-P12BH___%
Posx11BH = GenPolSV(P11BH(1),P12BH(1),'L',Tiempo,fs);
Posy11BH = GenPolSV(P11BH(2),P12BH(2),'L',Tiempo,fs);
Posz11BH = GenPolSV(P11BH(3),P12BH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang11BH = zeros(length(Posx11BH),4);
for i = 1:1:length(Posx11BH)
    ang11BH(i,:) = CInversa([Posx11BH(i) Posy11BH(i) Posz11BH(i)],53);
end

%___P12BH-P13BH___%
Posx12BH = GenPolSV(P12BH(1),P13BH(1),'L',Tiempo,fs);
Posy12BH = GenPolSV(P12BH(2),P13BH(2),'L',Tiempo,fs);
Posz12BH = GenPolSV(P12BH(3),P13BH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang12BH = zeros(length(Posx12BH),4);
for i = 1:1:length(Posx12BH)
    ang12BH(i,:) = CInversa([Posx12BH(i) Posy12BH(i) Posz12BH(i)],53);
end

%___P13BH-P14BH___%
Posx13BH = GenPolSV(P13BH(1),P14BH(1),'L',Tiempo,fs);
Posy13BH = GenPolSV(P13BH(2),P14BH(2),'L',Tiempo,fs);
Posz13BH = GenPolSV(P13BH(3),P14BH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang13BH = zeros(length(Posx13BH),4);
for i = 1:1:length(Posx13BH)
    ang13BH(i,:) = CInversa([Posx13BH(i) Posy13BH(i) Posz13BH(i)],53);
end

%___P14BH-P15BH___%
Posx14BH = GenPolSV(P14BH(1),P15BH(1),'L',Tiempo,fs);
Posy14BH = GenPolSV(P14BH(2),P15BH(2),'L',Tiempo,fs);
Posz14BH = GenPolSV(P14BH(3),P15BH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang14BH = zeros(length(Posx14BH),4);
for i = 1:1:length(Posx14BH)
    ang14BH(i,:) = CInversa([Posx14BH(i) Posy14BH(i) Posz14BH(i)],53);
end

%___P15BH-P16BH___%
Posx15BH = GenPolSV(P15BH(1),P16BH(1),'L',Tiempo,fs);
Posy15BH = GenPolSV(P15BH(2),P16BH(2),'L',Tiempo,fs);
Posz15BH = GenPolSV(P15BH(3),P16BH(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang15BH = zeros(length(Posx15BH),4);
for i = 1:1:length(Posx15BH)
    ang15BH(i,:) = CInversa([Posx15BH(i) Posy15BH(i) Posz15BH(i)],53);
end





angtot = vertcat(ang1BH,ang2BH,ang3BH,ang4BH,ang5BH,ang6BH,ang7BH,ang8BH,ang9BH,ang10BH,ang11BH,ang12BH,ang13BH,ang14BH,ang15BH);

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
title('Ángulos de articulaciones para la escritura de B en el plano horizontal')
ylabel('Ángulos(grados)')
xlabel('Tiempo(s)')


ang1mot = angtot(:, 1)+3;
ang2mot = angtot(:, 2)+10;
ang3mot = angtot(:, 3)+100;
ang4mot = angtot(:, 4)+105;

verificacion = vertcat(ang1mot,ang2mot,ang3mot,ang4mot)

for i = 1:1:length(verificacion)
    if verificacion(i) <0|verificacion>180;
        error('SINGULARIDAD');
    end
end

MRB = (horzcat(ang1mot,ang2mot,ang3mot,ang4mot))/180;


M_TotH = vertcat(MRA,MRM,MRB);

a = arduino('COM4','Uno');

ServoBase = servo(a,'D2'); % Pin Digital / Cita 1
ServoHombro = servo(a,'D6'); % Pin Digital / Cita 2
ServoHombroInv = servo(a,'D4'); % Pin Digital / Cita 2 Invertido
ServoCodo = servo(a,'D8'); % Pin Digital / Cita 3
ServoMuneca = servo(a,'D10'); % Pin Digital / Cita 4

[numFilas , numColumas] = size(M_TotH);

for i=1:numFilas
    writePosition( ServoBase  , MRB(i,1) ); % Angulo Motor Cita 1
    writePosition( ServoHombro, MRB(i,2) ); % Angulo Motor Cita 2
    writePosition( ServoHombroInv, 1-MRB(i,2) ); % Angulo Motor 2 Invertido
    writePosition( ServoCodo  , MRB(i,3) ); % Angulo Motor Cita 3
    writePosition( ServoMuneca, MRB(i,4) ); % Angulo Motor Cita 4
    pause(0.0001);
end

