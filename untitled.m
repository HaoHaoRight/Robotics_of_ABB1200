% 请求用户输入步长
h = input('请输入步长 h = ');

% 根据步长计算角度步长
theta_step = h; % 使用用户提供的步长作为角度步长

% Define the number of petals and the maximum radius
num_petals = 4; % For example, a 4-petal rose
max_radius = 5; % Maximum radius of the petals

% Initialize arrays to store the X and Y coordinates
X = [];
Y = [];

% Initialize the step counter
step_number = 0;

% Calculate the points on the petal curve
for theta = 0 : theta_step : 2*pi
    % Rose curve equation for a petal
    r = max_radius * cos(num_petals * theta);
    
    % Convert polar coordinates to Cartesian coordinates
    x = r * cos(theta);
    y = r * sin(theta);
    
    % Store the coordinates
    X(end+1) = x;
    Y(end+1) = y;
end

% Plot the curve
figure;
plot(X, Y, 'r-'); % Red color for the petal curve
axis equal; % Ensure equal scaling for both axes
grid on; % Turn on the grid
title('Full Quadrant Petal Curve Interpolation');
xlabel('X');
ylabel('Y');

% Hold on to plot the interpolation lines and step numbers
hold on;

% Interpolate between the points with straight lines and annotate steps
for i = 1:length(X)-1
    % Draw the line between points
    plot([X(i), X(i+1)], [Y(i), Y(i+1)], 'b-'); % Blue for interpolation lines
    
    % Increment the step counter
    step_number = step_number + 1;
    
    % Annotate the step number at the midpoint of the line
    mid_x = (X(i) + X(i+1)) / 2;
    mid_y = (Y(i) + Y(i+1)) / 2;
    text(mid_x, mid_y, num2str(step_number));
    
    % Pause to visualize the drawing process
    pause(0.1); 
end

% Hold off after plotting
hold off;

