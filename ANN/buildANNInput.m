function [X_all] = buildANNInput(data, dt)
    %% Number of samples
    N = size(data.accel, 1);

    %% 1. Acceleration magnitude
    acc_mag = sqrt(sum(data.accel.^2, 2));
    %% 2. Cumulative acceleration
    cum_acc_mag = cumsum(acc_mag) * dt;

    %% 3. Velocity magnitude
    vel_mag = sqrt(sum(data.vel.^2, 2));
    %% 4. Cumulative velocity
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

    %% Final ANN input matrix (Corrected assignment)
    X_all = [acc_mag, cum_acc_mag, vel_mag, cum_vel_mag, heading, cum_heading, vx_odo, cum_vx_odo, vy_odo, cum_vy_odo];
end
