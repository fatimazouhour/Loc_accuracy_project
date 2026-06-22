function X_all = buildANNInput(data, dt)

    %% Force correct orientation
    ax = data.accel(:,1); ax = ax(:);
    ay = data.accel(:,2); ay = ay(:);

    vx = data.vel(:,1); vx = vx(:);
    vy = data.vel(:,2); vy = vy(:);

    heading = data.heading(:);

    vx_odo = data.odoVel(:,1); vx_odo = vx_odo(:);
    vy_odo = data.odoVel(:,2); vy_odo = vy_odo(:);

    N = length(ax);

    %% Derived features (all N×1)
    cum_ax = cumsum(ax) * dt;
    cum_ay = cumsum(ay) * dt;

    cum_vx = cumsum(vx) * dt;
    cum_vy = cumsum(vy) * dt;

    cum_vx_odo = cumsum(vx_odo) * dt;
    cum_vy_odo = cumsum(vy_odo) * dt;

    %% Final feature matrix (13 features)
    X_all = [
        ax, ay, ...
        cum_ax, cum_ay, ...
        vx, vy, ...
        cum_vx, cum_vy, ...
        heading, ...
        vx_odo, vy_odo, ...
        cum_vx_odo, cum_vy_odo
    ];

end
