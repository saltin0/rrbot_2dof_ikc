clear;
clc;

% Simple velocity input position output model
s = tf('s');
Gs = 1/s;

% Design characteristics
Ts = 0.4;
tau = Ts/4;
K_in = 1/tau;

Ts_in = feedback(K_in*Gs,1);

Fpi = (s+20)/s;
Gs_out = Ts_in*Fpi;
rlocus(Gs_out)
K = 0.125;
Fpi = K*Fpi;
Ts_out = feedback(Fpi*Ts_in,1);
cont_signal = feedback(Fpi,Ts_in);
step(cont_signal)

margin(Gs)

