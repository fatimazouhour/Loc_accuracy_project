
%load data
Base = load('kf_input.mat');
K2   = load('kf2_output.mat');  kf2 = K2.kf2;

t     = Base.time_hist(:).';
N     = numel(t);
dt    = median(diff(t));
truth = [Base.true_x_hist(:).'; Base.true_y_hist(:).'];

gps_available = kf2.gps_available;        % reuse the EXACT EKF mask
if all(gps_available)
    warning(['gps_available is all true -> no outage, ANN never takes over. ' ...
             'Enable an outage in the EKF scripts so the mask propagates here.']);
end



%ann prediction
pos_ann = ann.predict(X).';                % 2 x N

%%fusion ann+kf2 see paper
a1_fuzzy = 0.5;                            % <-- FLS HOOK (fixed for now)
final = zeros(2,N);
for k = 1:N
    if gps_available(k)
        final(:,k) = kf2.pos(:,k);         % GPS up: KF-2 is the estimate
    else
        final(:,k) = a1_fuzzy*pos_ann(:,k) + (1-a1_fuzzy)*kf2.pos(:,k);  % Eq.5
    end
end

%RMSE
rmse = @(e) sqrt(mean(sum(e.^2,1)));
out  = ~gps_available;
fprintf('\n=== Combined position RMSE (m) ===\n');
if any(out)
    fprintf('-- during GPS OUTAGE --\n');
    fprintf('KF-2 alone : %.4f\n', rmse(truth(:,out)-kf2.pos(:,out)));
    fprintf('ANN alone  : %.4f\n', rmse(truth(:,out)-pos_ann(:,out)));
    fprintf('Fusion B   : %.4f   (a1_fuzzy=%.2f)\n', rmse(truth(:,out)-final(:,out)), a1_fuzzy);
end
fprintf('-- whole run --\n');
fprintf('KF-2     : %.4f\n', rmse(truth-kf2.pos));
fprintf('Fusion B : %.4f\n', rmse(truth-final));

%% plots
figure; hold on; grid on; axis equal
plot(truth(1,:),  truth(2,:),  'g-',  'LineWidth',2,   'DisplayName','Truth')
plot(kf2.pos(1,:),kf2.pos(2,:),'b-',  'LineWidth',1,   'DisplayName','KF-2')
plot(pos_ann(1,:),pos_ann(2,:),'-',   'Color',[.85 .33 .1],'DisplayName','ANN')
plot(final(1,:),  final(2,:),  'k--', 'LineWidth',1.8, 'DisplayName','Fusion B (ANN+KF-2)')
if any(out)
    plot(truth(1,out),truth(2,out),'rx','MarkerSize',5,'DisplayName','outage truth')
end
legend('Location','best'); xlabel('x (m)'); ylabel('y (m)')
title('Fusion B (paper Eq. 5): ANN + KF-2 during GPS outage')
