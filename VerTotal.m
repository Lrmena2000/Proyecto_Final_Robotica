clear all port;
clc;
clear all;
close all;
fs = 10;


Tiempo=5;


%---LETRA F Vertical---%
P0 =[]

P1FV = [-20 140 0]; 
P2FV = [-20 160 0];
P3FV = [-20 160 30];
P4FV = [-6 160 30];
P5FV = [-6 140 30];
P6FV = [-20 140 15];
P7FV = [-20 160 15];
P8FV = [-12 160 15];
P9FV = [-12 140 15];

%___P1FV-P2FV___%
Posx1FV = GenPolSV(P1FV(1),P2FV(1),'L',Tiempo,fs);
Posy1FV = GenPolSV(P1FV(2),P2FV(2),'L',Tiempo,fs);
Posz1FV = GenPolSV(P1FV(3),P2FV(3),'L',Tiempo,fs);


% Cinemática Inversa:
ang1FV = zeros(length(Posx1FV),4);
for i = 1:1:length(Posx1FV)
    ang1FV(i,:) = CInversa([Posx1FV(i) Posy1FV(i) Posz1FV(i)],15);
end


% Cinemática Inversa:
ang1FV = zeros(length(Posx1FV),4);
for i = 1:1:length(Posx1FV)
    ang1FV(i,:) = CInversa([Posx1FV(i) Posy1FV(i) Posz1FV(i)],15);
end

%DE AQUÍ AL MOTOR

%___P2FV-P3FV___%
Posx2FV = GenPolSV(P2FV(1),P3FV(1),'L',Tiempo,fs);
Posy2FV = GenPolSV(P2FV(2),P3FV(2),'L',Tiempo,fs);
Posz2FV = GenPolSV(P2FV(3),P3FV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang2FV = zeros(length(Posx2FV),4);
for i = 1:1:length(Posx2FV)
    ang2FV(i,:) = CInversa([Posx2FV(i) Posy2FV(i) Posz2FV(i)],15);
end

%___P3FV-P4FV___%
Posx3FV = GenPolSV(P3FV(1),P4FV(1),'L',Tiempo,fs);
Posy3FV = GenPolSV(P3FV(2),P4FV(2),'L',Tiempo,fs);
Posz3FV = GenPolSV(P3FV(3),P4FV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang3FV = zeros(length(Posx3FV),4);
for i = 1:1:length(Posx3FV)
    ang3FV(i,:) = CInversa([Posx3FV(i) Posy3FV(i) Posz3FV(i)],15);
end

%___P4FV-P5FV___%
Posx4FV = GenPolSV(P4FV(1),P5FV(1),'L',Tiempo,fs);
Posy4FV = GenPolSV(P4FV(2),P5FV(2),'L',Tiempo,fs);
Posz4FV = GenPolSV(P4FV(3),P5FV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang4FV = zeros(length(Posx4FV),4);
for i = 1:1:length(Posx4FV)
    ang4FV(i,:) = CInversa([Posx4FV(i) Posy4FV(i) Posz4FV(i)],15);
end

%___P5FV-P6FV___%
Posx5FV = GenPolSV(P5FV(1),P6FV(1),'L',Tiempo,fs);
Posy5FV = GenPolSV(P5FV(2),P6FV(2),'L',Tiempo,fs);
Posz5FV = GenPolSV(P5FV(3),P6FV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang5FV = zeros(length(Posx5FV),4);
for i = 1:1:length(Posx5FV)
    ang5FV(i,:) = CInversa([Posx5FV(i) Posy5FV(i) Posz5FV(i)],15);
end

%___P6FV-P7FV___%
Posx6FV = GenPolSV(P6FV(1),P7FV(1),'L',Tiempo,fs);
Posy6FV = GenPolSV(P6FV(2),P7FV(2),'L',Tiempo,fs);
Posz6FV = GenPolSV(P6FV(3),P7FV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang6FV = zeros(length(Posx6FV),4);
for i = 1:1:length(Posx6FV)
    ang6FV(i,:) = CInversa([Posx6FV(i) Posy6FV(i) Posz6FV(i)],15);
end

%___P7FV-P8FV___%
Posx7FV = GenPolSV(P7FV(1),P8FV(1),'L',Tiempo,fs);
Posy7FV = GenPolSV(P7FV(2),P8FV(2),'L',Tiempo,fs);
Posz7FV = GenPolSV(P7FV(3),P8FV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang7FV = zeros(length(Posx7FV),4);
for i = 1:1:length(Posx7FV)
    ang7FV(i,:) = CInversa([Posx7FV(i) Posy7FV(i) Posz7FV(i)],15);
end

%___P8FV-P9FV___%
Posx8FV = GenPolSV(P8FV(1),P9FV(1),'L',Tiempo,fs);
Posy8FV = GenPolSV(P8FV(2),P9FV(2),'L',Tiempo,fs);
Posz8FV = GenPolSV(P8FV(3),P9FV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang8FV = zeros(length(Posx8FV),4);
for i = 1:1:length(Posx8FV)
    ang8FV(i,:) = CInversa([Posx8FV(i) Posy8FV(i) Posz8FV(i)],15);
end




angtot = vertcat(ang1FV,ang2FV,ang3FV,ang4FV,ang5FV,ang6FV,ang7FV,ang8FV);

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
title('Ángulos de articulaciones para la escritura de F en el plano vertical')
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

MRF = (horzcat(ang1mot,ang2mot,ang3mot,ang4mot))/180;

Tiempo=5;


%---LETRA A Vertical---%
P0 =[]

P1AV = [4 140 0]; 
P2AV = [4 160 0];
P3AV = [11.5 160 30];
P4AV = [19 160 0];
P5AV = [19 140 0];
P6AV = [7.75 140 15];
P7AV = [7.75 160 15];
P8AV = [15.25 160 15];
P9AV = [15.25 140 15];

%___P1AV-P2AV___%
Posx1AV = GenPolSV(P1AV(1),P2AV(1),'L',Tiempo,fs);
Posy1AV = GenPolSV(P1AV(2),P2AV(2),'L',Tiempo,fs);
Posz1AV = GenPolSV(P1AV(3),P2AV(3),'L',Tiempo,fs);


% Cinemática Inversa:
ang1AV = zeros(length(Posx1AV),4);
for i = 1:1:length(Posx1AV)
    ang1AV(i,:) = CInversa([Posx1AV(i) Posy1AV(i) Posz1AV(i)],15);
end


% Cinemática Inversa:
ang1AV = zeros(length(Posx1AV),4);
for i = 1:1:length(Posx1AV)
    ang1AV(i,:) = CInversa([Posx1AV(i) Posy1AV(i) Posz1AV(i)],15);
end

%DE AQUÍ AL MOTOR

%___P2AV-P3AV___%
Posx2AV = GenPolSV(P2AV(1),P3AV(1),'L',Tiempo,fs);
Posy2AV = GenPolSV(P2AV(2),P3AV(2),'L',Tiempo,fs);
Posz2AV = GenPolSV(P2AV(3),P3AV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang2AV = zeros(length(Posx2AV),4);
for i = 1:1:length(Posx2AV)
    ang2AV(i,:) = CInversa([Posx2AV(i) Posy2AV(i) Posz2AV(i)],15);
end

%___P3AV-P4AV___%
Posx3AV = GenPolSV(P3AV(1),P4AV(1),'L',Tiempo,fs);
Posy3AV = GenPolSV(P3AV(2),P4AV(2),'L',Tiempo,fs);
Posz3AV = GenPolSV(P3AV(3),P4AV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang3AV = zeros(length(Posx3AV),4);
for i = 1:1:length(Posx3AV)
    ang3AV(i,:) = CInversa([Posx3AV(i) Posy3AV(i) Posz3AV(i)],15);
end

%___P4AV-P5AV___%
Posx4AV = GenPolSV(P4AV(1),P5AV(1),'L',Tiempo,fs);
Posy4AV = GenPolSV(P4AV(2),P5AV(2),'L',Tiempo,fs);
Posz4AV = GenPolSV(P4AV(3),P5AV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang4AV = zeros(length(Posx4AV),4);
for i = 1:1:length(Posx4AV)
    ang4AV(i,:) = CInversa([Posx4AV(i) Posy4AV(i) Posz4AV(i)],15);
end

%___P5AV-P6AV___%
Posx5AV = GenPolSV(P5AV(1),P6AV(1),'J',Tiempo,fs);
Posy5AV = GenPolSV(P5AV(2),P6AV(2),'J',Tiempo,fs);
Posz5AV = GenPolSV(P5AV(3),P6AV(3),'J',Tiempo,fs);

% Cinemática Inversa:
ang5AV = zeros(length(Posx5AV),4);
for i = 1:1:length(Posx5AV)
    ang5AV(i,:) = CInversa([Posx5AV(i) Posy5AV(i) Posz5AV(i)],15);
end

%___P6AV-P7AV___%
Posx6AV = GenPolSV(P6AV(1),P7AV(1),'L',Tiempo,fs);
Posy6AV = GenPolSV(P6AV(2),P7AV(2),'L',Tiempo,fs);
Posz6AV = GenPolSV(P6AV(3),P7AV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang6AV = zeros(length(Posx6AV),4);
for i = 1:1:length(Posx6AV)
    ang6AV(i,:) = CInversa([Posx6AV(i) Posy6AV(i) Posz6AV(i)],15);
end

%___P7AV-P8AV___%
Posx7AV = GenPolSV(P7AV(1),P8AV(1),'L',Tiempo,fs);
Posy7AV = GenPolSV(P7AV(2),P8AV(2),'L',Tiempo,fs);
Posz7AV = GenPolSV(P7AV(3),P8AV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang7AV = zeros(length(Posx7AV),4);
for i = 1:1:length(Posx7AV)
    ang7AV(i,:) = CInversa([Posx7AV(i) Posy7AV(i) Posz7AV(i)],15);
end

%___P8AV-P9AV___%
Posx8AV = GenPolSV(P8AV(1),P9AV(1),'L',Tiempo,fs);
Posy8AV = GenPolSV(P8AV(2),P9AV(2),'L',Tiempo,fs);
Posz8AV = GenPolSV(P8AV(3),P9AV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang8AV = zeros(length(Posx8AV),4);
for i = 1:1:length(Posx8AV)
    ang8AV(i,:) = CInversa([Posx8AV(i) Posy8AV(i) Posz8AV(i)],15);
end




angtot = vertcat(ang1AV,ang2AV,ang3AV,ang4AV,ang5AV,ang6AV,ang7AV,ang8AV);


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
title('Ángulos de articulaciones para la escritura de A en el plano vertical')
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


%---LETRA S Vertical---%
P0 =[]

P1SV = [29 140 0]; 
P2SV = [29 161 0];
P3SV = [45 161 0];
P4SV = [45 161 3];
P5SV = [45 161 12];
P6SV = [45 161 15];
P7SV = [33 161 15];
P8SV = [29 161 15];
P9SV = [29 161 27];
P10SV = [29 161 30];
P11SV = [45 161 30];
P12SV = [45 140 30];


%___P1SV-P2SV___%
Posx1SV = GenPolSV2(P1SV(1),P2SV(1),'L',Tiempo,fs);
Posy1SV = GenPolSV2(P1SV(2),P2SV(2),'L',Tiempo,fs);
Posz1SV = GenPolSV2(P1SV(3),P2SV(3),'L',Tiempo,fs);


% Cinemática Inversa:
ang1SV = zeros(length(Posx1SV),4);
for i = 1:1:length(Posx1SV)
    ang1SV(i,:) = CInversa([Posx1SV(i) Posy1SV(i) Posz1SV(i)],15);
end


% Cinemática Inversa:
ang1SV = zeros(length(Posx1SV),4);
for i = 1:1:length(Posx1SV)
    ang1SV(i,:) = CInversa([Posx1SV(i) Posy1SV(i) Posz1SV(i)],15);
end

%DE AQUÍ AL MOTOR

%___P2SV-P3SV___%
Posx2SV = GenPolSV2(P2SV(1),P3SV(1),'L',Tiempo,fs);
Posy2SV = GenPolSV2(P2SV(2),P3SV(2),'L',Tiempo,fs);
Posz2SV = GenPolSV2(P2SV(3),P3SV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang2SV = zeros(length(Posx2SV),4);
for i = 1:1:length(Posx2SV)
    ang2SV(i,:) = CInversa([Posx2SV(i) Posy2SV(i) Posz2SV(i)],15);
end

%___P3SV-P4SV___%
Posx3SV = GenPolSV2(P3SV(1),P4SV(1),'L',Tiempo,fs);
Posy3SV = GenPolSV2(P3SV(2),P4SV(2),'L',Tiempo,fs);
Posz3SV = GenPolSV2(P3SV(3),P4SV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang3SV = zeros(length(Posx3SV),4);
for i = 1:1:length(Posx3SV)
    ang3SV(i,:) = CInversa([Posx3SV(i) Posy3SV(i) Posz3SV(i)],15);
end

%___P4SV-P5SV___%
Posx4SV = GenPolSV2(P4SV(1),P5SV(1),'L',Tiempo,fs);
Posy4SV = GenPolSV2(P4SV(2),P5SV(2),'L',Tiempo,fs);
Posz4SV = GenPolSV2(P4SV(3),P5SV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang4SV = zeros(length(Posx4SV),4);
for i = 1:1:length(Posx4SV)
    ang4SV(i,:) = CInversa([Posx4SV(i) Posy4SV(i) Posz4SV(i)],15);
end

%___P5SV-P6SV___%
Posx5SV = GenPolSV2(P5SV(1),P6SV(1),'L',Tiempo,fs);
Posy5SV = GenPolSV2(P5SV(2),P6SV(2),'L',Tiempo,fs);
Posz5SV = GenPolSV2(P5SV(3),P6SV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang5SV = zeros(length(Posx5SV),4);
for i = 1:1:length(Posx5SV)
    ang5SV(i,:) = CInversa([Posx5SV(i) Posy5SV(i) Posz5SV(i)],15);
end

%___P6SV-P7SV___%
Posx6SV = GenPolSV2(P6SV(1),P7SV(1),'L',Tiempo,fs);
Posy6SV = GenPolSV2(P6SV(2),P7SV(2),'L',Tiempo,fs);
Posz6SV = GenPolSV2(P6SV(3),P7SV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang6SV = zeros(length(Posx6SV),4);
for i = 1:1:length(Posx6SV)
    ang6SV(i,:) = CInversa([Posx6SV(i) Posy6SV(i) Posz6SV(i)],15);
end

%___P7SV-P8SV___%
Posx7SV = GenPolSV2(P7SV(1),P8SV(1),'L',Tiempo,fs);
Posy7SV = GenPolSV2(P7SV(2),P8SV(2),'L',Tiempo,fs);
Posz7SV = GenPolSV2(P7SV(3),P8SV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang7SV = zeros(length(Posx7SV),4);
for i = 1:1:length(Posx7SV)
    ang7SV(i,:) = CInversa([Posx7SV(i) Posy7SV(i) Posz7SV(i)],15);
end

%___P8BH-P9BH___%
Posx8SV = GenPolSV2(P8SV(1),P9SV(1),'L',Tiempo,fs);
Posy8SV = GenPolSV2(P8SV(2),P9SV(2),'L',Tiempo,fs);
Posz8SV = GenPolSV2(P8SV(3),P9SV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang8SV = zeros(length(Posx8SV),4);
for i = 1:1:length(Posx8SV)
    ang8SV(i,:) = CInversa([Posx8SV(i) Posy8SV(i) Posz8SV(i)],15);
end

%___P9SV-P10SV___%
Posx9SV = GenPolSV2(P9SV(1),P10SV(1),'L',Tiempo,fs);
Posy9SV = GenPolSV2(P9SV(2),P10SV(2),'L',Tiempo,fs);
Posz9SV = GenPolSV2(P9SV(3),P10SV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang9SV = zeros(length(Posx9SV),4);
for i = 1:1:length(Posx9SV)
    ang9SV(i,:) = CInversa([Posx9SV(i) Posy9SV(i) Posz9SV(i)],15);
end

%___P10SV-P11SV___%
Posx10SV = GenPolSV2(P10SV(1),P11SV(1),'L',Tiempo,fs);
Posy10SV = GenPolSV2(P10SV(2),P11SV(2),'L',Tiempo,fs);
Posz10SV = GenPolSV2(P10SV(3),P11SV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang10SV = zeros(length(Posx10SV),4);
for i = 1:1:length(Posx10SV)
    ang10SV(i,:) = CInversa([Posx10SV(i) Posy10SV(i) Posz10SV(i)],15);
end

%___P11SV-P12SV___%
Posx11SV = GenPolSV2(P11SV(1),P12SV(1),'L',Tiempo,fs);
Posy11SV = GenPolSV2(P11SV(2),P12SV(2),'L',Tiempo,fs);
Posz11SV = GenPolSV2(P11SV(3),P12SV(3),'L',Tiempo,fs);

% Cinemática Inversa:
ang11SV = zeros(length(Posx11SV),4);
for i = 1:1:length(Posx11SV)
    ang11SV(i,:) = CInversa([Posx11SV(i) Posy11SV(i) Posz11SV(i)],15);
end







angtot = vertcat(ang1SV,ang2SV,ang3SV,ang4SV,ang5SV,ang6SV,ang7SV,ang8SV,ang9SV,ang10SV,ang11SV);

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
title('Ángulos de articulaciones para la escritura de S en el plano vertical')
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

MRS = (horzcat(ang1mot,ang2mot,ang3mot,ang4mot))/180;

M_TotV = vertcat(MRF,MRA,MRS)



