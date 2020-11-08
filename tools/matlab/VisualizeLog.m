log = LoadDump('/home/thomas/jetsoncar_logs/', '');

figure(1);
plot(log.time, log.encoder_front);
hold on;
plot(log.time, log.encoder_back);
hold off;
legend('Front', 'Back');

velocity_front = diff(log.encoder_front) ./ diff(log.time);
velocity_back = diff(log.encoder_back) ./ diff(log.time);
time2 = (log.time(2:end) + log.time(1:end-1)) / 2;

figure(2);
plot(time2, velocity_front);
hold on;
plot(time2, velocity_back);
hold off;
legend('Front', 'Back');
xlabel('Time [s]');
ylabel('Velocity [ticks/s]');

% %%
% dt = mean(diff(log.time));
% steps = 10;
% velocity = (log.encoder_back(1+steps:end) - log.encoder_back(1:end-steps)) / (2*dt);
% time2 = (log.time(1+steps:end) + log.time(1:end-steps)) / 2;
% figure(2);
% plot(time2, velocity);

%%
figure(3);
plot(log.time, log.throttle);

%%
encoder_ticks_rev = 12;
gear_ratio = 40;
wheel_diameter = 0.155; % meter

wheel_angle = 2*pi * diff(log.encoder_back) / (encoder_ticks_rev * gear_ratio);
angular_velocity = wheel_angle ./ diff(log.time);
velocity = angular_velocity * wheel_diameter/2;
time_velocity = (log.time(2:end) + log.time(1:end-1)) / 2;

figure(4);
subplot(2,1,1);
plot(time_velocity, angular_velocity);
ylabel('Velocity [rad/s]');
subplot(2,1,2);
plot(log.time, log.out_throttle);
xlabel('Time [s]');
ylabel('Throttle [%]');