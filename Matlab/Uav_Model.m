% Define physical parameters
mass = 1.5; % Mass of the UAV in kg
arm_length = 0.3; % Distance from the center to the propeller in meters
Ixx = 0.03; % Moment of inertia around the x-axis
Iyy = 0.03; % Moment of inertia around the y-axis
Izz = 0.06; % Moment of inertia around the z-axis
g = 9.81; % Gravitational acceleration in m/s^2

% Propeller parameters
propeller_thrust_coefficient = 1e-6; % Thrust coefficient (example value)
propeller_torque_coefficient = 1e-7; % Torque coefficient (example value)

% Initial conditions
initial_position = [0; 0; 0]; % Initial position in meters
initial_velocity = [0; 0; 0]; % Initial velocity in m/s
initial_orientation = [0; 0; 0]; % Initial orientation (roll, pitch, yaw) in radians
initial_angular_velocity = [0; 0; 0]; % Initial angular velocity in rad/s

% Simulation parameters
dt = 0.01; % Time step in seconds
simulation_time = 10; % Total simulation time in seconds
time = 0:dt:simulation_time; % Time vector

% Initialize state vectors
position = zeros(3, length(time));
velocity = zeros(3, length(time));
orientation = zeros(3, length(time));
angular_velocity = zeros(3, length(time));

% Set initial states
position(:, 1) = initial_position;
velocity(:, 1) = initial_velocity;
orientation(:, 1) = initial_orientation;
angular_velocity(:, 1) = initial_angular_velocity;

% Simulation loop
for k = 1:length(time)-1
    % Compute thrust and torques
    thrust = propeller_thrust_coefficient * (velocity(3, k)^2);
    torque = propeller_torque_coefficient * (angular_velocity(:, k).^2);
    
    % Update linear dynamics
    acceleration = [0; 0; -g] + [0; 0; thrust] / mass;
    velocity(:, k+1) = velocity(:, k) + acceleration * dt;
    position(:, k+1) = position(:, k) + velocity(:, k) * dt;
    
    % Update angular dynamics
    angular_acceleration = [torque(1) / Ixx; torque(2) / Iyy; torque(3) / Izz];
    angular_velocity(:, k+1) = angular_velocity(:, k) + angular_acceleration * dt;
    orientation(:, k+1) = orientation(:, k) + angular_velocity(:, k) * dt;
end

% Plot the position of the UAV
figure;
plot3(position(1, :), position(2, :), position(3, :));
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('UAV Position Over Time');
grid on;

% Visualize the UAV
figure;
for k = 1:10:length(time)
    clf;
    % Draw the propeller arm
    line([0 arm_length * cos(orientation(3, k))], ...
         [0 arm_length * sin(orientation(3, k))], ...
         [0 0], 'LineWidth', 3, 'Color', 'k');
    hold on;
    % Draw the UAV body
    plot3(position(1, k), position(2, k), position(3, k), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    axis equal;
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    zlabel('Z Position (m)');
    title('UAV Visualization');
    grid on;
    drawnow;
end
