% This script expects CSV files exported from Windarab containing:
% Imu LAT/LON [deg]
% Time [s]
% Steering Wheel Angle [deg],
% Speed [km/h]
% Yaw Rate [deg/s]

% Constant altitude
ALT = 32; 

% Crop your data [s]
T_MIN = 72;
T_MAX = 280;

% Geometric parameters
L = 1.5421; % Wheelbase
FBAL = 0.46; % Weight bias towards the front of the vehicle

% Initial guess for the steering ratio
STEER_RATIO_BEST_GUESS = 6.4286;

%% Parse data
data = readtable("/home/simonebondi/Desktop/16.csv");
data = data(data.xtime_S_ >= T_MIN & data.xtime_S_ <= T_MAX, :);

%% Convert position to LTP to plot it
lla = [data.IMU_LAT___, data.IMU_LONG___, repelem(ALT, size(data, 1))'];
lla0 = mean(lla);
[x,y,] = geodetic2enu(lla(:,1), lla(:,2), lla(:,3), lla0(1), lla0(2), lla0(3), wgs84Ellipsoid);
data.pos = [x, y];
clear x y lla lla0

%% Estimate Steer Ratio and Gyro bias from Steering Wheel Angle, Speed and Yaw Rate
in_data = [data.steer_Deg_ * (pi / 180), data.speed_km_h_ / 3.6];
out_data = data.IMU_RATE_OF_TURN_GYRZ___s_ * (pi / 180);

result = lsqcurvefit(@(x, xdata) (curv_from_steer(xdata(:,1), x(1), L, FBAL) .* xdata(:,2)) + x(2), [STEER_RATIO_BEST_GUESS, 0], in_data, out_data);
disp(result);

%% Plot the data and resulting predicted Yaw Rate
nexttile;
plot(data.xtime_S_, in_data(:,1));
title("Steering Wheel Angle [rad]");
nexttile;
plot(data.xtime_S_, in_data(:,2));
title("Speed [m/s]");
nexttile;
pred = (curv_from_steer(in_data(:,1), result(1), L, FBAL) .* in_data(:,2));
plot(data.xtime_S_, out_data(:,1));
hold on;
plot(data.xtime_S_, pred + result(2));
title("Yaw Rate [rad/s] - Recording vs Prediction");
legend('Recorded', 'Predicted');

%% Plot the predicted trajectory
nexttile;
speed = @(t) interp1(data.xtime_S_, in_data(:, 2), t);
u = @(t) interp1(data.xtime_S_, in_data(:, 1), t);

first = data.pos(1,:);
after_first = data.pos(find(vecnorm(data.pos - first, 2, 2) > 2, 1, 'first'), :);
delta = after_first - first;
theta0 = atan2(delta(2), delta(1));

theta = atan2(data.pos(5, 2) - data.pos(1, 2), data.pos(5, 1) - data.pos(1, 1));

odefun = @(t, y) speed(t) * [cos(y(3)); sin(y(3)); curv_from_steer(u(t), result(1), L, FBAL)];
y_ode = ode1(odefun, data.xtime_S_, [first(1); first(2); theta0]);

plot(data.pos(:,1), data.pos(:,2), 'Color', 'black');
hold on;
plot(y_ode(:,1), y_ode(:,2), 'blue');
title("Recorded vs predicted path");
legend('Recorded', 'Predicted');
