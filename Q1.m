function markov_localization()
    data = load('binary_map.mat');
    binary_map = data.output_matrix;
    map_shape = size(binary_map);

    %% Parameters
    move_prob = [0.05, 0.15, 0.60, 0.15, 0.05]; 
    move_steps = [1, 2, 3, 4, 5];
    initial_pos = [40, 20];  % [y, x] position my map was opposite scaled
    belief = ones(map_shape) / (map_shape(1) * map_shape(2));
    robot_pos = initial_pos;


    %% main cycle
    for cycle = 1:20
        robot_pos = move_robot(robot_pos, move_prob, move_steps, map_shape);
        belief = update_belief(belief, move_prob, move_steps, map_shape);
        lidar_data = simulate_lidar(robot_pos(1), robot_pos(2), binary_map);
        belief = update_belief_with_lidar(belief, lidar_data, binary_map);
        plot_belief(belief, binary_map, cycle, robot_pos);
        disp(['Cycle: ', num2str(cycle), ' - Max Belief: ', num2str(max(belief, [], 'all')), ' - Sum Belief: ', num2str(sum(belief, 'all'))]);
        if max(belief, [], 'all') >= 0.7 && cycle>3 %% to ensure loop will go for some time
            break;
        end
    end
end

function new_pos = move_robot(robot_pos, move_prob, move_steps, map_shape)
    direction = randi(4);  % Randomizer for direction
    move = randsample(move_steps, 1, true, move_prob);

    switch direction
        case 1
            new_pos = robot_pos + [0, -move];  % up
        case 2
            new_pos = robot_pos + [0, move];   % down
        case 3
            new_pos = robot_pos + [-move, 0];  % left
        case 4
            new_pos = robot_pos + [move, 0];   % right
    end

    new_pos = max(min(new_pos, map_shape), 1);  
end

function new_belief = update_belief(belief, move_prob, move_steps, map_shape)
    [map_height, map_width] = size(belief);
    new_belief = zeros(size(belief));
    
    for y = 1:map_height
        for x = 1:map_width
            for move_idx = 1:length(move_steps)
                move = move_steps(move_idx);
                prob = move_prob(move_idx);

                for dy = -move:move
                    for dx = -move:move
                        if abs(dx) + abs(dy) == move
                            new_x = x + dx;
                            new_y = y + dy;

                            if new_x > 0 && new_x <= map_width && new_y > 0 && new_y <= map_height
                                new_belief(new_y, new_x) = new_belief(new_y, new_x) + belief(y, x) * prob;
                            end
                        end
                    end
                end
            end
        end
    end

    new_belief = new_belief / sum(new_belief, 'all');  % Normalize
end

function lidar_distances = simulate_lidar(robot_y, robot_x, binary_map)
    max_range = 10; 
    num_beams = 360; 
    angle_increment = 2 * pi / num_beams; 
    [map_height, map_width] = size(binary_map); 
    lidar_distances = max_range * ones(1, num_beams); 

    for beam = 1:num_beams
        angle = (beam - 1) * angle_increment;
        for r = 1:max_range
            x = round(robot_x + r * cos(angle));
            y = round(robot_y + r * sin(angle));
            if x > 0 && x <= map_width && y > 0 && y <= map_height
                if binary_map(y, x) == 1
                    if rand() < 0.4  % 40% chance to detect correctly
                        lidar_distances(beam) = r;
                        break;
                    elseif rand() < 0.075  % 7.5% chance to detect in adjacent cell
                        % left cell
                        if x > 1 && binary_map(y, x-1) == 1
                            lidar_distances(beam) = r;
                            break;
                        % right cell
                        elseif x < map_width && binary_map(y, x+1) == 1
                            lidar_distances(beam) = r;
                            break;
                        %above cell
                        elseif y > 1 && binary_map(y-1, x) == 1
                            lidar_distances(beam) = r;
                            break;
                        % below cell
                        elseif y < map_height && binary_map(y+1, x) == 1
                            lidar_distances(beam) = r;
                            break;
                        end
                    end
                end
            end
        end
    end
end


function new_belief = update_belief_with_lidar(belief, lidar_data, binary_map)
    [map_height, map_width] = size(belief);
    new_belief = zeros(size(belief));

    for y = 1:map_height
        for x = 1:map_width
            simulated_lidar = simulate_lidar(y, x, binary_map);
            likelihood = calculate_likelihood(lidar_data, simulated_lidar);
            new_belief(y, x) = belief(y, x) * likelihood;
        end
    end

    new_belief = new_belief / sum(new_belief, 'all');  
end

function likelihood = calculate_likelihood(actual, simulated)
    likelihood = 1;
    for i = 1:length(actual)
        if actual(i) == simulated(i)
            likelihood = likelihood * 0.7;
        elseif abs(actual(i) - simulated(i)) <= 1
            %% ı added those 0.2 for 0.1 by experiments
            likelihood = likelihood * 0.2; % Reduced confidence
        else
            %% ı added those 0.2 for 0.1 by experiments
            likelihood = likelihood * 0.1; % Increased uncertainty
        end
    end
end

function plot_belief(belief, binary_map, cycle, robot_pos)
    subplot(1, 2, 1);
    imagesc(binary_map);
    colormap(gray);
    hold on;
    plot(robot_pos(2), robot_pos(1), 'bo', 'MarkerSize', 10, 'LineWidth', 3);
    hold off;
    title(['Map and Robot Position - Cycle ', num2str(cycle)]);
    
    subplot(1, 2, 2);
    imagesc(belief);
    colormap(hot);
    colorbar;
    hold on;
    plot(robot_pos(2), robot_pos(1), 'bo', 'MarkerSize', 10, 'LineWidth', 3);
    hold off;
    title(['Belief Distribution - Cycle ', num2str(cycle)]);
    
    drawnow;
end


