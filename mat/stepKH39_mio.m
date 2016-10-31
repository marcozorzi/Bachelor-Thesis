clear all
%close all
clc

Np=200; % numero di passi/giro del motore KH39
alfa_p= 2*pi/Np; % angolo di passo
p=0.025; % passo della vite
r_eq=lead/(2*pi); % raggio equivalente del sistema vite-madrevite
J=3.5706e-04; % momento di inerzia equivalente
TL=1.2639; % coppia di carico costante dovuta all'attrito
T=2; % coppia motrice richiesta al motore
alfa_s=(T-TL)/J; % specifica di accelerazione angolare
a=alfa_s/alfa_p;  % a=108.22, costante di aggiornamento periodo di passo
acc= alfa_s*r_eq; % accelerazione scalare
per(1)=1/800; % periodo iniziale di commutazione delle fasi
fs(1)=1/per(1); % frequenza di passo iniziale
fs_fin=2440; % frequenza di passo finale
ii=1;
velocita(1)=r_eq*alfa_p/per(1);
tempo(1)=per(1);
spazio(1)=0; % Spazio percorso dal carico
fs_cor(1)=0;
while (fs_cor<fs_fin)
    ii = ii+1;
    per(ii)=per(ii-1)*(1-a*per(ii-1)^2);
    per_cor(ii)=per(ii-1)/(1+a*per(ii-1)^2); % senza approssimazione di Mac Laurin
    fs(ii)=1/per(ii);
    fs_cor(ii)=1/per_cor(ii); % senza approssimazione di Mac Laurin
    spazio(ii)=spazio(ii-1)+alfa_p*r_eq;
    velocita(ii)= velocita(ii-1)+acc*per_cor(ii-1);
    tempo(ii)=tempo(ii-1)+per_cor(ii);
end

ii
figure(7); % Frequenza di passo in funzione del tempo
    plot(tempo,fs,tempo,fs_cor,'r');
    grid;
    xlabel('Tempo (s)');
    ylabel('Passi al secondo (pps)');
figure(8);
    plot(tempo, velocita);
    grid;
    xlabel('Tempo (s)');
    ylabel('Velocit? lineare (m/s)');
figure(9); plot(tempo,per_cor, tempo, per );
    grid
    xlabel('Tempo (s)');
    ylabel('Tempo di commutazione (s)');