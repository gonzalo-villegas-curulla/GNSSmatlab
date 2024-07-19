% ============  INTRO   ============

fs = 1;
refFrame = "NED";

% Longitude Latitude Altitude
lla0 = [42.2825 -71.343 53.0352];
N = 100; % num samps

% Allocations
pos = zeros(N, 3);
vel = zeros(N, 3);
time = (0:N-1) ./ fs;


% ===========================================================

gps = gpsSensor("SampleRate", fs, "ReferenceLocation", lla0, ...
    "ReferenceFrame", refFrame);

gnss = gnssSensor("SampleRate", fs, "ReferenceLocation", lla0, ...
    "ReferenceFrame", refFrame);

% POS(x, y, z) = llaGPS( 1-3 data cols )

% Generate outputs.
[llaGPS, velGPS] = gps(pos, vel);

% Visualize position.
figure();

subplot(3, 1, 1); plot(time, llaGPS(:,1));
title('Latitude'); ylabel('degrees'); xlabel('s');

subplot(3, 1, 2);
plot(time, llaGPS(:,2))
title('Longitude'); ylabel('degrees'); xlabel('s'); 

subplot(3, 1, 3);
plot(time, llaGPS(:,3)); 
title('Altitude'); ylabel('m');  xlabel('s'); 




% Visualize velocity.
figure();
plot(time, velGPS);
title('Velocity'); legend('X', 'Y', 'Z'); ylabel('m/s'); xlabel('s');

%% ================= Simulate: gnssSensor() ===================

% Generate outputs.
[llaGNSS, velGNSS] = gnss(pos, vel);

% Same again: position (lla) and velocity, this time for GNSS instead.


% Visualize positon.
figure(); 

subplot(311);
plot(time, llaGNSS(:,1));
title('Latitude'); ylabel('degrees'); xlabel('s');

subplot(312);
plot(time, llaGNSS(:,2));
title('Longitude');  ylabel('degrees');  xlabel('s');

subplot(313);
plot(time, llaGNSS(:,3)); 
title('Altitude'); ylabel('m'); xlabel('s');

% Visualize velocity.
figure();
plot(time, velGNSS);
title('Velocity'); legend('X', 'Y', 'Z'); ylabel('m/s'); xlabel('s');

%% ============================= Precision =====================

% "Dilution of precision" of GPS with respect to GNSS, horizontal and
% vertical

% Set the RNG seed to reproduce results. 
rng('default');

% Specify the start time of the simulation.
initTime = datetime(2020, 4, 20, 18, 10, 0, "TimeZone", "America/New_York");

% Create the GNSS receiver model.
gnss = gnssSensor("SampleRate", fs, "ReferenceLocation", lla0, ...
    "ReferenceFrame", refFrame, "InitialTime", initTime);

% Obtain the receiver status. 
[~, ~, status] = gnss(pos, vel);
disp(status(1))

% HDOP = horizontal dilution of precision
% VDOP = vertical idem

hdops = vertcat(status.HDOP);

figure();
plot(time, vertcat(status.HDOP));
title('HDOP'); ylabel('m'); xlabel('s');


% ========================== Num of Satellites ===============

% Consider a variation of the number of satellites in view

% Find expected sample index for a change in the number of satellites in
% view.
[~, satChangeIdx] = max(abs(diff(hdops)));


% Visualize the satellite geometry before the change in HDOP.
satAz = status(satChangeIdx).SatelliteAzimuth;
satEl = status(satChangeIdx).SatelliteElevation;
numSats = numel(satAz);
skyplot(satAz, satEl);
title(sprintf('Satellites in View: %d\nHDOP: %.4f', ...
    numSats, hdops(satChangeIdx)));



% Visualize the satellite geometry after the
% change in HDOP.
satAz = status(satChangeIdx+1).SatelliteAzimuth;
satEl = status(satChangeIdx+1).SatelliteElevation;
numSats = numel(satAz);
skyplot(satAz, satEl);
title(sprintf('Satellites in View: %d\nHDOP: %.4f', ...
    numSats, hdops(satChangeIdx+1)));



% ===== A bit of mathematics to wrap it up =======

% HDOP and VDOP: diagonal elements, measured covariance of GNSS receiver position estimates.
% Kalman filter of other sensor measurements.

% Convert HDOP and VDOP to a measurement covariance matrix.
hdop = status(1).HDOP;
vdop = status(1).VDOP;
measCov = diag([hdop.^2/2, hdop.^2/2, vdop.^2]);
disp(measCov)