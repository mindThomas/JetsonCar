log = LoadDump('/home/thomas/jetsoncar_logs/', 'motor_sysid_8V0.txt');

encoder_ticks_rev = 12;
gear_ratio = 40;
wheel_diameter = 0.155; % meter

wheel_angle = 2*pi * diff(log.encoder_back) / (encoder_ticks_rev * gear_ratio);
angular_velocity = wheel_angle ./ diff(log.time);
velocity = angular_velocity * wheel_diameter/2;
time_velocity = (log.time(2:end) + log.time(1:end-1)) / 2;

figure(1);
subplot(2,1,1);
plot(time_velocity, 3.6*velocity);
ylabel('Velocity [km/hr]');
subplot(2,1,2);
plot(log.time, log.out_throttle);
xlabel('Time [s]');
ylabel('Throttle [%]');

%% Fit positive exponential
%deadband = 0.1;
throttle = (log.out_throttle(2:end)+log.out_throttle(1:end-1))/2;
%idx = find(throttle >= deadband);
idx = find(angular_velocity >= 0.1);
[a, b, c] = fitExponentialStep(throttle(idx), angular_velocity(idx));

deadband = c;
% a = max(angular_velocity(idx));
% b = 1;
% c = 0;

figure(2);
plot(throttle(idx), angular_velocity(idx), '.');
xlabel('Throttle');
ylabel('Angular velocity');

vline(deadband, 'r');
vline(-deadband, 'r');
yl = get(gca,'ylim');
hold on;
%fplot(@(x) 22*(1-exp(7*(deadband-x))));
fplot(@(x) a*(1-exp(b*(c-x))));
hold off;
ylim(yl);
xlim([deadband, 1]);

params_positive = [a, b, c];

%% Fit negative exponential
throttle = (log.out_throttle(2:end)+log.out_throttle(1:end-1))/2;
idx = find(angular_velocity <= -0.1);
[a, b, c] = fitExponentialStep(-throttle(idx), -angular_velocity(idx));
a = -a;
b = -b;
c = -c;

deadband = c;
% a = max(angular_velocity(idx));
% b = 1;
% c = 0;

figure(2);
plot(throttle(idx), angular_velocity(idx), '.');
xlabel('Throttle');
ylabel('Angular velocity');

yl = get(gca,'ylim');
hold on;
fplot(@(x) a*(1-exp(b*(c-x))));
hold off;
ylim(yl);
xlim([-1, deadband]);

params_negative = [a, b, c];

%% Inverse mapping for Feedforward
% throttle -> speed
% speed = a*(1-exp(b*(c-throttle)))
% speed -> throttle
% speed/a = 1-exp(b*(c-throttle))
% 1 - speed/a = exp(b*(c-throttle))
% ln(1 - speed/a) = b*(c-throttle)
% ln(1 - speed/a) / b = c-throttle
% c - ln(1 - speed/a) / b = throttle