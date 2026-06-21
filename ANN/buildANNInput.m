function [X_all] = buildANNInput(data, dt)
% buildANNInput
%
% This function builds the ANN input matrix from processed sensor data.
%
%===========================================================
% INPUT DATA STRUCTURE
%
% data.accel   -> Nx3 accelerometer measurements
%                 [ax ay az]
%
% data.vel     -> Nx2 velocity estimates from INS
%                 [vx vy]
%
% data.heading -> Nx1 heading/yaw angle
%                 [psi]
%
% data.odoVel  -> Nx2 odometer velocities
%                 [vx_odo vy_odo]
%
% dt           -> sampling time [s]
%
% Example project mapping:
%
% data.accel   = sensors.accelB;
% data.vel     = vel_ins;
% data.heading = yaw_ins;
% data.odoVel  = sensors.odoVelB;
%
% X = buildANNInput(data, dt);
%
%===========================================================
% OUTPUT
%
% X -> Nx10 ANN input matrix
%
% Each row is one time sample:
%
% X(k,:) = [
%   acc_mag(k),
%   cum_acc_mag(k),
%   vel_mag(k),
%   cum_vel_mag(k),
%   heading(k),
%   cum_heading(k),
%   vx_odo(k),
%   cum_vx_odo(k),
%   vy_odo(k),
%   cum_vy_odo(k)
% ];
%
%===========================================================

    %% Number of samples
    N = size(data.accel, 1);

    %% Check input dimensions

    if size(data.accel, 2) ~= 3
        error('data.accel must be Nx3: [ax ay az]');
    end

    if size(data.vel, 2) ~= 2
        error('data.vel must be Nx2: [vx vy]');
    end

    if size(data.odoVel, 2) ~= 2
        error('data.odoVel must be Nx2: [vx_odo vy_odo]');
    end

    if length(data.heading) ~= N
        error('data.heading must have the same number of rows as data.accel');
    end

    %% 1. Acceleration magnitude
    % acc_mag = sqrt(ax^2 + ay^2 + az^2)

    acc_mag = sqrt(sum(data.accel.^2, 2));

    %% 2. Cumulative acceleration magnitude

    cum_acc_mag = cumsum(acc_mag) * dt;

    %% 3. Velocity magnitude
    % vel_mag = sqrt(vx^2 + vy^2)

    vel_mag = sqrt(sum(data.vel.^2, 2));

    %% 4. Cumulative velocity magnitude

    cum_vel_mag = cumsum(vel_mag) * dt;

    %% 5. Heading

    heading = data.heading(:);

    %% 6. Cumulative heading

    cum_heading = cumsum(heading) * dt;

    %% 7. Odometer x velocity

    vx_odo = data.odoVel(:, 1);

    %% 8. Cumulative odometer x velocity

    cum_vx_odo = cumsum(vx_odo) * dt;

    %% 9. Odometer y velocity

    vy_odo = data.odoVel(:, 2);

    %% 10. Cumulative odometer y velocity

    cum_vy_odo = cumsum(vy_odo) * dt;

    %% Final ANN input matrix

    X = [ ...
        acc_mag, ...
        cum_acc_mag, ...
        vel_mag, ...
        cum_vel_mag, ...
        heading, ...
        cum_heading, ...
        vx_odo, ...
        cum_vx_odo, ...
        vy_odo, ...
        cum_vy_odo];
end
