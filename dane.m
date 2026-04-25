%% =========================================================================
% Zadanie eliminacyjne konkursu RCDC - Skrypt Inicjalizacyjny
% Zespół (nazwa)
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

%% 4. Parametry pojazdu:
% =========================================================================
% GENERATOR TRAJEKTORII: LEWY PAS, PRAWY PAS ORAZ JAZDA PROSTO
% =========================================================================

% --- Wspólne parametry dla wszystkich 3 trajektorii ---
L = 50;              % Długość samego manewru zmiany pasa [m]
L_calkowite = 80;    % Całkowita długość trajektorii (manewr + jazda prosto po manewrze) [m]
W = 3.5;             % Szerokość pasa [m] (+ dla lewego, - dla prawego)
dx = 0.5;            % Rozdzielczość (odstępy między punktami X) [m]

% Wektory osi X (identyczne dla zmiany pasa w lewo i w prawo)
X_manewr = 0:dx:L;
X_end = (L+dx):dx:L_calkowite;
X_pelne = [X_manewr, X_end]; % Kompletna oś X od 0 do 80m

% -------------------------------------------------------------------------
% 1. LEWY PAS (Zaczyna manewr od razu, kończy na Y = +3.5)
% -------------------------------------------------------------------------
Y_manewr_lewy = (W / 2) * (1 - cos(pi * X_manewr / L)); 
Y_end_lewy = W * ones(size(X_end));
Y_pelne_lewy = [Y_manewr_lewy, Y_end_lewy];

Waypoints_Lewy = [X_pelne', Y_pelne_lewy']; % Macierz do Simulinka (Nx2)

% -------------------------------------------------------------------------
% 2. PRAWY PAS (Zaczyna manewr od razu, kończy na Y = -3.5)
% -------------------------------------------------------------------------
Y_manewr_prawy = (-W / 2) * (1 - cos(pi * X_manewr / L)); 
Y_end_prawy = -W * ones(size(X_end));
Y_pelne_prawy = [Y_manewr_prawy, Y_end_prawy];

Waypoints_Prawy = [X_pelne', Y_pelne_prawy']; % Macierz do Simulinka (Nx2)

% -------------------------------------------------------------------------
% 3. JAZDA PROSTO (Utrzymanie obecnego pasa, Y = 0)
% -------------------------------------------------------------------------
Y_pelne_prosto = zeros(size(X_pelne));

Waypoints_Prosto = [X_pelne', Y_pelne_prosto']; % Macierz do Simulinka (Nx2)

% =========================================================================
% WIZUALIZACJA DO WERYFIKACJI
% =========================================================================
figure('Name', 'Generowane Trajektorie');
hold on; grid on; axis equal;

% Rysowanie krzywych
plot(Waypoints_Lewy(:,1), Waypoints_Lewy(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'Lewy Pas');
plot(Waypoints_Prosto(:,1), Waypoints_Prosto(:,2), 'k--', 'LineWidth', 2, 'DisplayName', 'Prosto');
plot(Waypoints_Prawy(:,1), Waypoints_Prawy(:,2), 'r-', 'LineWidth', 2, 'DisplayName', 'Prawy Pas');

% Estetyka wykresu
title('Zestawienie trajektorii dla bloku decyzyjnego w Simulinku');
xlabel('Dystans wzdłużny X [m]');
ylabel('Przesunięcie poprzeczne Y [m]');
legend('Location', 'best');
ylim([-5 5]);
%% Otwarcie i uruchomienie modelu
open_system("model");
