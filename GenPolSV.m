function [Pos,t,Vel,Ace,Mat_Coef] = GenPolSV(Pos0,Posf,mov,tf,fs,PVias)
% INPUTS:
% Pos0: Posición inicial
% Posf: Posición final
% CantInt: Cantidad de puntos intermedios deseados
% tf: Tiempo que se espera para realizar todo el recorrido
% fs: Frecuencia de muestreo de los datos
% mov: Tipo de movimiento a realizar
% PVias: Puntos vías (opcional, en caso de utilizarse, no se tomará en
% cuenta el parámetro de puntos intermedios)

indicador =0;
if Pos0 == Posf
    Prov = Pos0;
    Pos0 = 3;
    Posf = 5;
    indicador =1;
else
    indicador =0;
end

% Determinación del tipo de trayectoria deseada
if mov =='L'
    CantInt = 10;
else
    CantInt = 0;
end

% Establecimeinto de casos por si se introducen o no puntos vía
switch nargin
    case 6
        Puntos = horzcat(Pos0,PVias,Posf);
    otherwise
       Puntos = Pos0:(Posf-Pos0)/(CantInt+1):Posf;
end

ts = tf/(length(Puntos)-1); % Duración de cada segmento
% m y n, matrices para la creación del vector de condiciones del sistema de ecuaciones
m = zeros(1,2*(length(Puntos)-1));
n = zeros(1,length(m));

q = 1; 
for i =1:1:length(m)
    if mod(i,2) ==0
        q=q+1;
    end
    m(i) = Puntos(q);
end

% p: Primera parte del vector de coeficientes de tiempo (t) para el sistema
% de ecuaciones
p = zeros(length(m),2*length(m));

q =-1;
l = -1;

for i = 1:1:length(m)
     if mod(i,2) ~=0
         q = q+1;
         p(i,i+2*q)=1;
     else
         l = l+1;
         for h = 0:1:3
             p(i,h+i-1+2*l) =ts^h;
         end
     end
end


q = -1;
% z: segunda parte de la matriz de coeficientes de tiempo (t) para el
% sistema de ecuaciones
z = zeros(length(m),2*length(m));
vec1 =[2 6*ts 0 0 -2];
for i = 1:1:length(Puntos)-1
    tt = 0;
    if i ==1
        q = q+4;
        z(i,i+1) = 1;
    else
        for s=q:1:q+4
            tt=tt+1;
            z(i,s)=vec1(tt);
        end
         q = q+4;
    end
end

q = 2;
vec2 = [1 2*ts 3*ts^2 0 -1];
for g = i+1:1:length(m)
    tt =0;
    if g == length(m)
        for s = q:1:q+2
            tt = tt+1;
            z(g,s) = vec2(tt);
        end
    else
        for s=q:1:q+4
            tt = tt+1;
            z(g,s)=vec2(tt);
        end
        q = q+4;
    end
end

vecfinal1 = [p;z];  %Matriz de coeficientes de t
vecfinal2 = [m';n']; %Matriz de condiciones

% Resolución del sistema de ecuaiones por factorización LU
[L,U]=lu(vecfinal1);
d = L\vecfinal2;
coef = round(U\d,4);

%Matriz de coeficientes (a) para cada tramo de la trayectoria

Mat_Coef = zeros(length(coef)/4,4);

i = 0;
ex = 1;
while i ~= length(coef)/4
    for q = 1:1:4
       Mat_Coef(ex,q) = coef(q+4*i,1);
    end
   i = i+1;
   ex = ex+1;
end



ta = 0:1/fs:ts;
%Evaluación para cada una de las subtrayectorias
tam = size(Mat_Coef);
tam = tam(1);
t_temp = zeros(tam,length(ta));


for i = 1:1:tam
    t_temp(i,:) = polyval(fliplr(Mat_Coef(i,:)),ta);
end

if length(Puntos)==2
    Pos = t_temp;
else
    % Creación de un solo vector con todos los datos de las trayectorias
    [M,N1] = size(t_temp);
    Pos = zeros(1,round(fs*2*ts+1));
    cont = 1;
    for n = 1:M     % Bucle de filas

        if n==M
            N =N1;
        else
            N = N1-1;
        end
        for m = 1:N % Bucle de columnas
            Pos(cont) = t_temp(n,m);

            cont = cont + 1;

        end

    end
end



% Creación de matriz de coeficientes para la velocidad
[m,n] = size(Mat_Coef);
Mat_Coef2 =zeros(m,n-1);

for i = 1:1:m
    for k = 2:1:n
        switch k
            case 2
                Mat_Coef2(i,k-1) = Mat_Coef(i,k);
            case 3
                Mat_Coef2(i,k-1) = Mat_Coef(i,k)*2;
            case 4
                Mat_Coef2(i,k-1) = Mat_Coef(i,k)*3;
        end
    end
end

%Evaluación para velocidades de cada una de las subtrayectorias
tam = size(Mat_Coef2);
tam = tam(1);
t_temp = zeros(tam,length(ta));


for i = 1:1:tam
    t_temp(i,:) = polyval(fliplr(Mat_Coef2(i,:)),ta);
end


if length(Puntos)==2
    Vel = t_temp;
else
    % Creación de un solo vector con todos los datos de las velocidades
    [M,N1] = size(t_temp);
    Vel = zeros(1,round(fs*2*ts+1));
    cont = 1;
    for n = 1:M     % Bucle de filas

        if n==M
            N =N1;
        else
            N = N1-1;
        end
        for m = 1:N % Bucle de columnas
            Vel(cont) = t_temp(n,m);

            cont = cont + 1;

        end

    end
end

% Creación de matriz de coeficientes para la aceleración
[m,n] = size(Mat_Coef2);
Mat_Coef3 =zeros(m,n-1);

for i = 1:1:m
    for k = 2:1:n
        switch k
            case 2
                Mat_Coef3(i,k-1) = Mat_Coef2(i,k);
            case 3
                Mat_Coef3(i,k-1) = Mat_Coef2(i,k)*2;
        end
    end
end


%Evaluación para aceleraciones de cada una de las subtrayectorias
tam = size(Mat_Coef3);
tam = tam(1);
t_temp = zeros(tam,length(ta));


for i = 1:1:tam
    t_temp(i,:) = polyval(fliplr(Mat_Coef3(i,:)),ta);
end

if length(Puntos)==2
    Ace = t_temp;
else
    % Creación de un solo vector con todos los datos de las aceleraciones
    [M,N1] = size(t_temp);
    Ace = zeros(1,round(fs*2*ts+1));
    cont = 1;
    for n = 1:M     % Bucle de filas

        if n==M
            N =N1;
        else
            N = N1-1;
        end
        for m = 1:N % Bucle de columnas
            Ace(cont) = t_temp(n,m);

            cont = cont + 1;

        end

    end
end
if indicador == 1
    Vel = zeros(1,length(Pos));
    Ace = zeros(1,length(Pos));
    for i =1:1:length(Pos)
        Pos(i) = Prov;
    end
end


% Creación del vector de tiempo
t = 0:tf/length(Pos):tf-tf/length(Pos);