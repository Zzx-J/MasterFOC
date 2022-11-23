function [Ta, Tb, Tc, N] = SVPWM(d,q,theta)
Vdc=24; 
nPhase = 3;
rad = theta/4000*2*pi;
Valpha = d*cos(rad)-q*sin(rad);
Vbeta = d*sin(rad)+q*cos(rad);
% from Valpha Vbeta calculates the 
% T0 - the idle time
% T1 - the time at Vi
% T2 - the time at V(i+1) = Vj

nPhase = floor(nPhase);
if nPhase <3
    nPhase = 3;
end

Valpha = Valpha / Vdc;
Vbeta = Vbeta/Vdc;

angle = atan(Vbeta/Valpha);
if Valpha<0
    angle = angle + pi;
end
if angle <0
    angle = angle + 2*pi;
end

i = floor(angle/pi*nPhase);
Vi = exp(1j*pi*i/nPhase);
Vj = exp(1j*pi*(i+1)/nPhase);
%fprintf("sector %d\n", i);
%fprintf("Vi %f,%f | Vj %f, %f\n",real(Vi),imag(Vi),real(Vj),imag(Vj));

Vm = (Vi+Vj)/2;
Vmabs = abs(Vm);
Vout = Valpha + 1j*Vbeta;
%Vabs = abs(Vout);
%fprintf("Vm %f,%f Vmabs %f\n",real(Vm),imag(Vm),Vmabs);
Vout_para = real(Vout * conj(Vm))/Vmabs;
%fprintf("Vout %f,%f Vout_para %f\n",real(Vout),imag(Vout),Vout_para);
if Vout_para > Vmabs
    Vout = Vout /Vout_para *Vmabs;
    Vout_para  = Vmabs;
end
%fprintf("Vout %f,%f Vout_para %f\n",real(Vout),imag(Vout),Vout_para);

T0 = 1- Vout_para / Vmabs;
Vom = Vout / Vout_para *Vmabs - Vm;
T2 = (real(Vom/(Vj-Vi))+0.5) * (1-T0);
T1 = 1-T0-T2;
N = mod(i,2*nPhase);
%fprintf("Vom %f,%f\n",real(Vom),imag(Vom));
%fprintf("T0 %f,T2 %f,T1 %f,N %d\n",T0,T2,T1,N);

Tall = [T0/4 T1/2+T0/4 T2/2+T0/4 T1/2+T2/2+T0/4];

switch (N)
case 0
    Ta = Tall(1);
    Tb = Tall(2);
    Tc = Tall(4);
case 1
    Ta = Tall(3);
    Tb = Tall(1);
    Tc = Tall(4);
case 2
    Ta = Tall(4);
    Tb = Tall(1);
    Tc = Tall(2);
case 3
    Ta = Tall(4);
    Tb = Tall(3);
    Tc = Tall(1);
case 4
    Ta = Tall(2);
    Tb = Tall(4);
    Tc = Tall(1);
case 5
    Ta = Tall(1);
    Tb = Tall(4);
    Tc = Tall(3);
otherwise
    Ta = 0;
    Tb = 0;
    Tc = 0;
            
end

end