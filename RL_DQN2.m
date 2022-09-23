%% clear 
clearvars;

load('IDM_data.mat');
rng(2);

%% initialization - driving parameters

L_car_x = 5; % length of a car in x
p_leading_max = 20;
p_leading_min = L_car_x;
p_adjacent_max = 10;
p_adjacent_min = -10;

N_leading_car = 0; % 0 or 1
N_adjacent_cars = 1;
N_cars = 1 + N_leading_car + N_adjacent_cars; % number of cars in simulation
% including ego car
% the order must be ego, leading (if any), adjacent cars

delta_t = 0.1; % simulation step size in seconds
N_t = 100;

%conversion = 2.237; % converted from miles per hour to meters per second
conversion = 3.6; % converted from km per hour to meters per second
s_car_min = 60/conversion; % min speed
s_car_max = 120/conversion; % max speed
p_car_x_MAX = 40;

a_max = 2; % max acceleration meters per second^2
tau = 1; 

% action from any state is 1 (a_max), 2 (0), and 3 (-a_max)
action = [a_max, 0, -a_max];

K=length(action);

%% initialization - Q-learning parameters

Temp_0 = 1.1; % simulated annealing rate
Temp = Temp_0.^(-(1:N_t));

r_min = -1;
r_max = 10;

gamma = 0.8;

%% initialization - deep network parameters

test_set = 0.2;

M=1e6; % number of trials
C=100; % number of batch iterations for every weight update

% Calculate Number of Input and Output NodesActivations
nbrOfInputNodes = 2*N_cars; %=Dimention of Any Input Samples
% input nodes are v0, v1, d1, v2, d2, ..., a

nbrOfNeuronsInEachHiddenLayer = [30 30]; %linear:[4] - circle:[10] - wave,spirals:[10 10]

nbrOfOutUnits = 1;
mode = [-1 0 0 2]; %0 for Unipolar, 1 for Bipolar, 2 for linear
% the first number is dummy

nbrOfLayers = 2 + length(nbrOfNeuronsInEachHiddenLayer);
nbrOfNodesPerLayer = [nbrOfInputNodes nbrOfNeuronsInEachHiddenLayer nbrOfOutUnits];

% Adding the Bias as Nodes with a fixed Activation of 1
nbrOfNodesPerLayer(1:end-1) = nbrOfNodesPerLayer(1:end-1) + 1;


learningRate = 0.15;

enable_resilient_gradient_descent = 1; %1 for enable, 0 for disable
learningRate_plus = 1.2;
learningRate_negative = 0.5;
deltas_start = 0.9;
deltas_min = 10^-6;
deltas_max = 50;

enable_decrease_learningRate = 1; %1 for enable decreasing, 0 for disable
learningRate_decreaseValue = 0.0001;
min_learningRate = 0.05;

enable_learningRate_momentum = 0; %1 for enable, 0 for disable
momentum_alpha = 0.05;

input_scaled_range = 1; % input variable is to be scaled to [0,input_scaled_range]

%% Initialize Random Wieghts Matrices
Weights = cell(1, nbrOfLayers-1); %Weights connecting bias nodes with previous layer are useless, but to make code simpler and faster
Delta_Weights = cell(1, nbrOfLayers-1);
ResilientDeltas = Delta_Weights; % Needed in case that Resilient Gradient Descent is used
for i = 1:length(Weights)
    Weights{i} = 2*rand(nbrOfNodesPerLayer(i), nbrOfNodesPerLayer(i+1))-1; %RowIndex: From Node Number, ColumnIndex: To Node Number
    Weights{i}(:,1) = 0; %Bias nodes weights with previous layer (Redundant step)
    Delta_Weights{i} = zeros(nbrOfNodesPerLayer(i), nbrOfNodesPerLayer(i+1));
    ResilientDeltas{i} = deltas_start*ones(nbrOfNodesPerLayer(i), nbrOfNodesPerLayer(i+1));
end
Old_Delta_Weights_for_Momentum = Delta_Weights;
Old_Delta_Weights_for_Resilient = Delta_Weights;

NodesActivations = cell(1, nbrOfLayers);
for i = 1:length(NodesActivations)
    NodesActivations{i} = zeros(1, nbrOfNodesPerLayer(i));
end
NodesBackPropagatedErrors = NodesActivations; %Needed for Backpropagation Training Backward Pass



%% iteration
for m=1:M    
    
    % drop cars

    p_car_x = zeros(N_cars,N_t);
    s_car = zeros(N_cars,N_t);
    a_car = zeros(N_cars,N_t);
    
    p_car_x(1,1)=0; % ego car starts at x-position = 0
    
    s_car(1,1)= s_car_min + (s_car_max-s_car_min)*rand; % scalar speed, in meters per second; 
    
%    p_car_x(2,1)= p_leading_min + (p_leading_max-p_leading_min)*rand;
    p_car_x(2,1)= p_adjacent_min + (p_adjacent_max-p_adjacent_min)*rand;
            
%    s_car(2,1)= s_car(1,1); % leading car always has the same speed as the ego car
    %s_car(3,1)= s_car_min + (s_car_max-s_car_min)*rand; % scalar speed, in meters per second; 
%    s_range = 0.1;
%    s_car(2,1) = s_car(1,1) * (1+(rand-0.5)/0.5*s_range);

% input speed data for other cars

%   for test_set = 0.2, rand*(1-0.2) generates a random number [0,0.8)
    index_s_car_sample = floor((rand*(1-test_set))*N_cars_IDM*N_samples_IDM)*N_t+1;
    s_car(2,:) = s_car_x_samples(index_s_car_sample:index_s_car_sample+N_t-1);
 
    for t=1:N_t
        
        %% select the next move
        Q = zeros(K,1);
        NodesActivations{1} = zeros(1, nbrOfInputNodes+1);
        NodesActivations{1}(1) = 1;
        NodesActivations{1}(2) = (s_car(1,t)-s_car_min)/(s_car_max-s_car_min)*input_scaled_range;
        for i=2:N_cars
            NodesActivations{1}(2*(i-1)+1) = (s_car(i,t)-s_car_min)/(s_car_max-s_car_min)*input_scaled_range;
            NodesActivations{1}(2*(i-1)+2) = (p_car_x(i,t)+p_car_x_MAX)/(2*p_car_x_MAX)*input_scaled_range;
        end
        
        for k = 1:K           
            NodesActivations{1}(nbrOfInputNodes+1) = (action(k)-min(action))/(max(action)-min(action))*input_scaled_range;
            for Layer = 2:nbrOfLayers
                NodesActivations{Layer} = NodesActivations{Layer-1}*Weights{Layer-1};
                NodesActivations{Layer} = Activation_func(NodesActivations{Layer}, mode(Layer));
                if (Layer ~= nbrOfLayers) %Because bias nodes don't have weights connected to previous layer
                    NodesActivations{Layer}(1) = 1;
                end
            end
            
            Q(k) = NodesActivations{nbrOfLayers};
        end
        
        prob = exp(Q/Temp(t));
        prob = prob/sum(prob);
        cprob=0;
        pick = rand;
        for k=1:K
            cprob = cprob + prob(k);
            if (pick<=cprob)
                a=k;
                break;
            end
        end
        
        %% make a move
        a_car(1,t) = action(a);
        
        p_car_x(:,t+1) = p_car_x(:,t) + s_car(:,t)*delta_t; 
        p_car_x(:,t+1) = p_car_x(:,t+1) - p_car_x(1,t+1);
    
        % only update ego car speed
        % other cars' speed follow inputted data
        s_car(1,t+1) = max(min(s_car(1,t) + a_car(1,t)*delta_t,s_car_max),s_car_min);        
            
        %% check reward
        r = r_min;
                    
        % see whether the current state is a safe state
        flag = ones(N_cars,1);
        flag(1)=0;
        for i=2:N_cars 

            if ((N_leading_car > 0) && (i==2))  % leading does exist
                % leading car
                d_el = p_car_x(i,t+1)-p_car_x(1,t+1);
                if (d_el < L_car_x)
                    disp('error: d_el < L_car_x');
                end
                d_el_tau2 = d_el + (s_car(i,t+1) - s_car(1,t+1))*tau/2 - 1/2*a_max*(tau/2)^2;
                if (d_el_tau2 >= L_car_x)
                    flag(i) = 0;
                end
            else
                % adjacent car
                d_ei = p_car_x(i,t+1)-p_car_x(1,t+1);

                d_ei_minus_tau2 = d_ei + (s_car(i,t+1) - s_car(1,t+1))*tau/2 - 1/2*a_max*(tau/2)^2;
                d_ei_minus_tau = d_ei + (s_car(i,t+1) - s_car(1,t+1))*tau - 1/2*a_max*(tau)^2;
                if ((d_ei_minus_tau2 >= L_car_x) && (d_ei_minus_tau >= L_car_x))
                    if (d_ei_minus_tau >= max(0,(s_car(1,t+1)^2-(s_car(i,t+1)-a_max*tau)^2)/2/a_max)+L_car_x)
                        flag(i) = 0;
                    end
                end

                d_ei_plus_tau2 = d_ei + (s_car(i,t+1) - s_car(1,t+1))*tau/2 + 1/2*a_max*(tau/2)^2;
                d_ei_plus_tau = d_ei + (s_car(i,t+1) - s_car(1,t+1))*tau + 1/2*a_max*(tau)^2;
                if ((-d_ei_plus_tau2 >= L_car_x) && (-d_ei_plus_tau >= L_car_x))
                    if (-d_ei_plus_tau >= max(0,(-s_car(1,t+1)^2+(s_car(i,t+1)+a_max*tau)^2)/2/a_max)+L_car_x)
                        flag(i) = 0;
                    end
                end
            end
        end

        % if the current state is already a safe one, no need to search further
        if (max(flag) == 0)
            r = r_max;          
        end           
                 
        %% accumulate Delta_Weights
        Q = zeros(K,1);
        NodesActivations{1} = zeros(1, nbrOfInputNodes+1);
        NodesActivations{1}(1) = 1;
        NodesActivations{1}(2) = (s_car(1,t+1)-s_car_min)/(s_car_max-s_car_min)*input_scaled_range;
        for i=2:N_cars
            NodesActivations{1}(2*(i-1)+1) = (s_car(i,t+1)-s_car_min)/(s_car_max-s_car_min)*input_scaled_range;
            NodesActivations{1}(2*(i-1)+2) = (p_car_x(i,t+1)+p_car_x_MAX)/(2*p_car_x_MAX)*input_scaled_range;
        end
        
        for k = 1:K           
            NodesActivations{1}(nbrOfInputNodes+1) = (action(k)-min(action))/(max(action)-min(action))*input_scaled_range;
            for Layer = 2:nbrOfLayers
                NodesActivations{Layer} = NodesActivations{Layer-1}*Weights{Layer-1};
                NodesActivations{Layer} = Activation_func(NodesActivations{Layer}, mode(Layer));
                if (Layer ~= nbrOfLayers) %Because bias nodes don't have weights connected to previous layer
                    NodesActivations{Layer}(1) = 1;
                end
            end
            
            Q(k) = NodesActivations{nbrOfLayers};
        end
        Q1_max = max(Q);
        
        NodesActivations{1} = zeros(1, nbrOfInputNodes+1);
        NodesActivations{1}(1) = 1;
        NodesActivations{1}(2) = (s_car(1,t+1)-s_car_min)/(s_car_max-s_car_min)*input_scaled_range;
        for i=2:N_cars
            NodesActivations{1}(2*(i-1)+1) = (s_car(i,t+1)-s_car_min)/(s_car_max-s_car_min)*input_scaled_range;
            NodesActivations{1}(2*(i-1)+2) = (p_car_x(i,t+1)+p_car_x_MAX)/(2*p_car_x_MAX)*input_scaled_range;
        end
        NodesActivations{1}(nbrOfInputNodes+1) = action(a);
        for Layer = 2:nbrOfLayers
            NodesActivations{Layer} = NodesActivations{Layer-1}*Weights{Layer-1};
            NodesActivations{Layer} = Activation_func(NodesActivations{Layer}, mode(Layer));
            if (Layer ~= nbrOfLayers) %Because bias nodes don't have weights connected to previous layer
                NodesActivations{Layer}(1) = 1;
            end
        end
            
        % Backward Pass Errors Storage
        % (As gradient of the bias nodes are zeros, they won't contribute to previous layer errors nor delta_weights)
        NodesBackPropagatedErrors{nbrOfLayers} =  (r+gamma*Q1_max) - NodesActivations{nbrOfLayers};
        for Layer = nbrOfLayers-1:-1:1
            gradient = Activation_func_drev(NodesActivations{Layer+1}, mode(Layer+1));
            for node=1:length(NodesBackPropagatedErrors{Layer}) % For all the Nodes in current Layer
                NodesBackPropagatedErrors{Layer}(node) =  sum( NodesBackPropagatedErrors{Layer+1} .* gradient .* Weights{Layer}(node,:) );
            end
        end
        
        % Backward Pass Delta Weights Calculation (Before multiplying by learningRate)
        for Layer = nbrOfLayers:-1:2
            derivative = Activation_func_drev(NodesActivations{Layer}, mode(Layer));    
            Delta_Weights{Layer-1} = Delta_Weights{Layer-1} + NodesActivations{Layer-1}' * (NodesBackPropagatedErrors{Layer} .* derivative);
        end
 
        %% check to see whether the game ends
        if (r == r_max)
            % stop the current trial
            break;
        end
        
        % continue
    end
    
    %% Apply resilient gradient descent or/and momentum to the delta_weights even C trials
    if (mod(m,C)==0)
    %    disp(['m=',num2str(m)]);
        if (enable_resilient_gradient_descent) % Handle Resilient Gradient Descent
    %        if (mod(Epoch,200)==0) %Reset Deltas
    %            for Layer = 1:nbrOfLayers-1
    %                ResilientDeltas{Layer} = learningRate*Delta_Weights{Layer};
    %            end
    %        end
            for Layer = 1:nbrOfLayers-1
                mult = Old_Delta_Weights_for_Resilient{Layer} .* Delta_Weights{Layer};
                ResilientDeltas{Layer}(mult > 0) = ResilientDeltas{Layer}(mult > 0) * learningRate_plus; % Sign didn't change
                ResilientDeltas{Layer}(mult < 0) = ResilientDeltas{Layer}(mult < 0) * learningRate_negative; % Sign changed
                ResilientDeltas{Layer} = max(deltas_min, ResilientDeltas{Layer});
                ResilientDeltas{Layer} = min(deltas_max, ResilientDeltas{Layer});

                Old_Delta_Weights_for_Resilient{Layer} = Delta_Weights{Layer};

                Delta_Weights{Layer} = sign(Delta_Weights{Layer}) .* ResilientDeltas{Layer};
            end
        end
        if (enable_learningRate_momentum) %Apply Momentum
            for Layer = 1:nbrOfLayers-1
                Delta_Weights{Layer} = learningRate*Delta_Weights{Layer} + momentum_alpha*Old_Delta_Weights_for_Momentum{Layer}; 
            end
            Old_Delta_Weights_for_Momentum = Delta_Weights;
        end
        if (~enable_learningRate_momentum && ~enable_resilient_gradient_descent)
            for Layer = 1:nbrOfLayers-1
                Delta_Weights{Layer} = learningRate * Delta_Weights{Layer};
            end
        end

        % Backward Pass Weights Update
        for Layer = 1:nbrOfLayers-1
            Weights{Layer} = Weights{Layer} + Delta_Weights{Layer};
        end

        % Resetting Delta_Weights to Zeros
        for Layer = 1:length(Delta_Weights)
            Delta_Weights{Layer} = 0 * Delta_Weights{Layer};
        end

        % Decrease Learning Rate
        if (enable_decrease_learningRate)
            learningRate = learningRate - learningRate_decreaseValue;
            learningRate = max(min_learningRate, learningRate);
        end
    end
end

save('RL_QDN2_workspace');