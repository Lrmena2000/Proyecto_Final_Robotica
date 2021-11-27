function Angulos = CInversa(Pos,Orientacion)
% INPUTS
X = Pos(1);
Y = Pos(2);
Z = Pos(3);
Orientacion = deg2rad(Orientacion);
C3 = (X^2+Y^2+Z^2-125^2-50^2-45^2)/(2*125*50);

if abs(C3)>1
    error('Cos(z3) es mayor que 1 o menor que -1, cambie los valores de entrada')
else
    S3 = -sqrt(1-C3^2);
end

if abs(S3)>1
    error('Sen(z3) es mayor que 1 o menor que -1, cambie valores de entrada')
else
    ang3 = atan2(S3,C3);
end

m = sqrt(X^2+Y^2-45^2);

S2 = (Z*125-m*50*S3+Z*50*C3)/(125^2+2*125*50*C3+50^2);

if abs(S2)>1
    error('Sen(z2) es mayor que 1 o menor que -1, cambie valores de entrada')
end

C2 = (m*125+Z*50*S3+m*50*C3)/(125^2+2*125*50*C3+50^2);

if abs(C2)>1
    error('Cos(z2) es mayor que 1 o menor que -1, cambie valores de entrada')
else
    ang2 = atan2(S2,C2);
end

S1 = (X*45+Y*m)/(45^2+m^2);

if abs(S1)>1
    error('Sen(z1) es mayor que 1 o menor que -1, cambie valores de entrada')
end

C1 = (X*m-Y*45)/(45^2+m^2);

if abs(C1)>1
    error('Cos(z1) es mayor que 1 o menor que -1, cambie valores de entrada')
else
    ang1 = atan2(S1,C1);
end

if Y==0
    ang1 = 0;
end

ang4 = -Orientacion -ang2-ang3;

ang1 = rad2deg(ang1);
ang2 = rad2deg(ang2);
ang3 = rad2deg(ang3);
ang4 = rad2deg(ang4);

Angulos = [ang1 ang2 ang3 ang4];


