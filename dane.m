%% =========================================================================
% Zadanie eliminacyjne konkursu RCDC - Skrypt Inicjalizacyjny
% Transfer function Team
% mateusz Łaś
% Paweł Kowalewski
% Tomasz Mazur
% =========================================================================
clear; clc;


%% 1. Parametry początkowe symulacji
v_ego = 30;        % [m/s] Prędkość początkowa naszego pojazdu
v_lead = 25;       % [m/s] Prędkość pojazdu poprzedzającego 
d_init = 200;      % [m] Dystans początkowy.

%% 2. Parametry czujników i filtru 
radar_max_range = 150;  % [m] Zasięg przedniego radaru 
fs = 100;               % [Hz] Częstotliwość próbkowania
tau_radar = 0.02;       % [s] Stała czasowa filtru radaru. 

%% 3. Inne parametry
ttc_threshold = 1.5;    % [s] Krytyczny czas do kolizji 
delay = 8;
dist_ref = 100;


%% Otwarcie i uruchomienie modelu
open_system("model");
