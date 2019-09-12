%*********************************************************************************************
% Author: Zhiyang Ru and Yong Wang
% Last Edited: 19/08/2019
% Email: zhiyang.ru@csu.edu.cn; ywang@csu.edu.cn

% Reference: Y. Wang, Z.-Y. Ru, K. Wang, and P.-Q. Huang. Joint deployment and task scheduling
% optimization for large-scale mobile users in multi-UAV enabled mobile edge computing.
% IEEE Transactions on Cybernetics, in press. DOI: 10.1109/TCYB.2019.2935466
%*********************************************************************************************

clc
clear all
close all

format long

time_max = 30;   % the maximum number of the independent runs
FEs_max = 10000;   % the maximum number of fitness evaluations
d_min = 10;   % the minimum distance between UAVs

% Select which problem to be tested
for problem_index = 1 : 10
    
    switch problem_index
        
        case 1
            
            M = 100;
            
            lu_p = [0, 0, 1.6e7, 81920; 320, 320, 1.6e9, 8192000];   % lu_p: the upper and lower bounds of the parameters
            task = reshape(ones(M, 1) * lu_p(1, :) + rand(M, 4) .* (ones(M, 1) * (lu_p(end, :) - lu_p(1, :))), 1, 4 * M);
            
            lu_v = [0, 0; 320, 320];   % lu_v: the upper and lower bounds of the variables
            
        case 2
            
            M = 200;
            
            lu_p = [0, 0, 1.6e7, 81920; 450, 450, 1.6e9, 8192000];   % lu_p: the upper and lower bounds of the parameters
            task = reshape(ones(M, 1) * lu_p(1, :) + rand(M, 4) .* (ones(M, 1) * (lu_p(end, :) - lu_p(1, :))), 1, 4 * M);
            
            lu_v = [0, 0; 450, 450];   % lu_v: the upper and lower bounds of the variables
            
        case 3
            
            M = 300;
            
            lu_p = [0, 0, 1.6e7, 81920; 550, 550, 1.6e9, 8192000];   % lu_p: the upper and lower bounds of the parameters
            task = reshape(ones(M, 1) * lu_p(1, :) + rand(M, 4) .* (ones(M, 1) * (lu_p(end, :) - lu_p(1, :))), 1, 4 * M);
            
            lu_v = [0, 0; 550, 550];   % lu_v: the upper and lower bounds of the variables
            
        case 4
            
            M = 400;
            
            lu_p = [0, 0, 1.6e7, 81920; 640, 640, 1.6e9, 8192000];   % lu_p: the upper and lower bounds of the parameters
            task = reshape(ones(M, 1) * lu_p(1, :) + rand(M, 4) .* (ones(M, 1) * (lu_p(end, :) - lu_p(1, :))), 1, 4 * M);
            
            lu_v = [0, 0; 640, 640];   % lu_v: the upper and lower bounds of the variables
            
        case 5
            
            M = 500;
            
            lu_p = [0, 0, 1.6e7, 81920; 710, 710, 1.6e9, 8192000];   % lu_p: the upper and lower bounds of the parameters
            task = reshape(ones(M, 1) * lu_p(1, :) + rand(M, 4) .* (ones(M, 1) * (lu_p(end, :) - lu_p(1, :))), 1, 4 * M);
            
            lu_v = [0, 0; 710, 710];   % lu_v: the upper and lower bounds of the variables
            
        case 6
            
            M = 600;
            
            lu_p = [0, 0, 1.6e7, 81920; 780, 780, 1.6e9, 8192000];   % lu_p: the upper and lower bounds of the parameters
            task = reshape(ones(M, 1) * lu_p(1, :) + rand(M, 4) .* (ones(M, 1) * (lu_p(end, :) - lu_p(1, :))), 1, 4 * M);
            
            lu_v = [0, 0; 780, 780];   % lu_v: the upper and lower bounds of the variables
            
        case 7
            
            M = 700;
            
            lu_p = [0, 0, 1.6e7, 81920; 840, 840, 1.6e9, 8192000];   % lu_p: the upper and lower bounds of the parameters
            task = reshape(ones(M, 1) * lu_p(1, :) + rand(M, 4) .* (ones(M, 1) * (lu_p(end, :) - lu_p(1, :))), 1, 4 * M);
            
            lu_v = [0, 0; 840, 840];   % lu_v: the upper and lower bounds of the variables
            
        case 8
            
            M = 800;
            
            lu_p = [0, 0, 1.6e7, 81920; 900, 900, 1.6e9, 8192000];   % lu_p: the upper and lower bounds of the parameters
            task = reshape(ones(M, 1) * lu_p(1, :) + rand(M, 4) .* (ones(M, 1) * (lu_p(end, :) - lu_p(1, :))), 1, 4 * M);
            
            lu_v = [0, 0; 900, 900];   % lu_v: the upper and lower bounds of the variables
            
        case 9
            
            M = 900;
            
            lu_p = [0, 0, 1.6e7, 81920; 950, 950, 1.6e9, 8192000];   % lu_p: the upper and lower bounds of the parameters
            task = reshape(ones(M, 1) * lu_p(1, :) + rand(M, 4) .* (ones(M, 1) * (lu_p(end, :) - lu_p(1, :))), 1, 4 * M);
            
            lu_v = [0, 0; 950, 950];   % lu_v: the upper and lower bounds of the variables
            
        case 10
            
            M = 1000;
            
            lu_p = [0, 0, 1.6e7, 81920; 1000, 1000, 1.6e9, 8192000];   % lu_p: the upper and lower bounds of the parameters
            task = reshape(ones(M, 1) * lu_p(1, :) + rand(M, 4) .* (ones(M, 1) * (lu_p(end, :) - lu_p(1, :))), 1, 4 * M);
            
            lu_v = [0, 0; 1000, 1000];   % lu_v: the upper and lower bounds of the variables
            
    end
    
    % Main body
    for time = time_max : time_max
        
        tic
        
        N = ceil(M / 10);
        
        % Initialize the population
        pop_UAV = deployment_initialization(d_min, N, lu_v);
        pop = [pop_UAV, greedy_algorithm(M, task, N, pop_UAV)];
        
        % Evaluate the population
        pop_fitness = evaluation(M, task, N, pop);
        FEs = 1;
        
        % Record the best results
        best_objective_values = ones(FEs_max, 3);
        
        best_variable_set = [N, pop];
        best_objective_values(FEs, :) = -pop_fitness;
        best_objective_value = -pop_fitness;
        
        flag = 0;
        infeasibility = 0;
        
        if pop_fitness(2) == 0
            
            infeasibility = 0;
            
        else
            
            infeasibility = infeasibility + 1;
            
        end
        
        while FEs < FEs_max
            
            % Eliminate the individual from the population and record the best results
            while FEs < FEs_max && flag == 0 && infeasibility == 0
                
                pop_UAV0 = reshape(pop(1 : 2 * N), N, 2);
                pop_X1 = pop_UAV0(:, 1) * ones(1, N);
                pop_X2 = pop_X1';
                pop_Y1 = pop_UAV0(:, end) * ones(1, N);
                pop_Y2 = pop_Y1';
                d = sqrt((pop_X1 - pop_X2) .^ 2 + (pop_Y1 - pop_Y2) .^ 2);
                d(d == 0) = 1e100;
                
                N = N - 1;
                
                d0_min = min(min(d));
                [row, column] = find(d == d0_min, 1);
                d(row, column) = 1e100;
                d(column, row) = 1e100;
                if min(d(row, :)) < min(d(column, :))
                    
                    pop_UAV0(row, :) = [];
                    
                else
                    
                    pop_UAV0(column, :) = [];
                    
                end
                
                pop_UAV = reshape(pop_UAV0, 1, 2 * N);
                pop = [pop_UAV, greedy_algorithm(M, task, N, pop_UAV)];
                
                pop_fitness = evaluation(M, task, N, pop);
                FEs = FEs + 1;
                
                if pop_fitness(end) > -best_objective_value(end)
                    
                    best_variable_set = [N, pop];
                    best_objective_value = -pop_fitness;
                    
                end
                
                best_objective_values(FEs, :) = -pop_fitness;
                
                if pop_fitness(2) == 0
                    
                    infeasibility = 0;
                    
                else
                    
                    infeasibility = infeasibility + 1;
                    
                end
                
            end
            
            if FEs == FEs_max
                
                break
                
            end
            
            % Generate individuals that constitute the set Q_UAV0
            pop_UAV0 = reshape(pop(1 : 2 * N), N, 2);
            Q_UAV0 = DE_operations(lu_v, pop_UAV0);
            
            % Update the population and record the best results
            for Q_index = 1 : N
                
                pop_index_set = 1 : N;
                pop_index = ceil(rand * (N - 1));
                pop_index_set(pop_index) = [];
                
                S_UAV0 = pop_UAV0;
                S_UAV0(pop_index, :) = Q_UAV0(Q_index, :);
                
                if all(sqrt(sum((S_UAV0(pop_index, :) - S_UAV0(pop_index_set, :)) .^ 2, 2)) >= d_min)
                    
                    S_UAV = reshape(S_UAV0, 1, 2 * N);
                    S = [S_UAV, greedy_algorithm(M, task, N, S_UAV)];
                    
                    S_fitness = evaluation(M, task, N, S);
                    FEs = FEs + 1;
                    
                    if S_fitness(end) > pop_fitness(end)
                        
                        pop = S;
                        pop_fitness = S_fitness;
                        
                        if pop_fitness(end) > -best_objective_value(end)
                            
                            best_variable_set = [N, pop];
                            best_objective_value = -pop_fitness;
                            
                        end
                        
                    end
                    
                    best_objective_values(FEs, :) = -pop_fitness;
                    
                    if FEs == FEs_max
                        
                        break
                        
                    end
                    
                    if pop_fitness(2) < 0
                        
                        infeasibility = infeasibility + 1;
                        
                        if N < ceil(M / 10) && infeasibility == 1000
                            
                            flag = 1;
                            
                            N = N + 1;
                            
                            pop = best_variable_set(2 : end);
                            pop_fitness = -best_objective_value;
                            
                            break
                            
                        end
                        
                    end
                    
                    if flag == 0 && pop_fitness(2) == 0
                        
                        infeasibility = 0;
                        
                        break
                        
                    end
                    
                end
                
            end
            
        end
        
        t = toc;
        
    end
    
end
