clearvars;
L_car_x = 5; % length of a car in x

rng(2);
N_samples = 1000;
p_leading_max = 20;
p_leading_min = L_car_x;
p_adjacent_max = 10;
p_adjacent_min = -10;

a_max = 2; % max acceleration meters per second^2

N_leading_car = 1; % 0 or 1
N_adjacent_cars = 1;
N_cars = 1 + N_leading_car + N_adjacent_cars; % number of cars in simulation
% including ego car
% the order must be ego, leading (if any), adjacent cars

delta_t = 0.1; % simulation step size in seconds
T_max=100;
N_t = 100;

a_adjacent = [0, -a_max, a_max];

T_samples = zeros(length(a_adjacent), N_samples);

for sample = 1:N_samples

    % drop cars

    p_car_x = zeros(N_cars,N_t);
    s_car = zeros(N_cars,N_t);
    a_car = zeros(N_cars,N_t);

    %conversion = 2.237; % converted from miles per hour to meters per second
    conversion = 3.6; % converted from km per hour to meters per second
    s_car_min = 60/conversion; % min speed
    s_car_max = 120/conversion; % max speed

    tau = 1; 

    p_car_x(1,1)=0; % ego car starts at x-position = 0
    
    s_car(1,1)= s_car_min + (s_car_max-s_car_min)*rand; % scalar speed, in meters per second; 
    
    p_car_x(2,1)= p_leading_min + (p_leading_max-p_leading_min)*rand;
    p_car_x(3,1)= p_adjacent_min + (p_adjacent_max-p_adjacent_min)*rand;
            
    s_car(2,1)= s_car(1,1); % leading car always has the same speed as the ego car
    %s_car(3,1)= s_car_min + (s_car_max-s_car_min)*rand; % scalar speed, in meters per second; 
    s_range = 0.1;
    s_car(3,1) = s_car(1,1) * (1+(rand-0.5)/0.5*s_range);
    
    if (N_adjacent_cars == 2)
        s_car(4,1) = s_car(3,1);
        a_car(4,1:t_adjacent) = a_adjacent;    
        
        while (1)
            p_car_x(4,1)= p_adjacent_min + (p_adjacent_max-p_adjacent_min)*rand;
            if (abs(p_car_x(4,1) - p_car_x(3,1)) > L_car_x)
                break;
            end
        end
    end
    
%    a_adjacent = a_max;
%
%    if (a_adjacent > 0)
%        t_adjacent = floor(s_car_max - s_car(3,1))/(a_adjacent*delta_t);
%    elseif (a_adjacent < 0)
%        t_adjacent = floor(s_car(3,1)-s_car_min)/(-a_adjacent*delta_t);
%    end
%    a_car(3,1:t_adjacent) = a_adjacent;    
    
    for a_adjacent_index = 1:length(a_adjacent)
        tic;
        
        a_car = zeros(N_cars,N_t);
        
        if (a_adjacent(a_adjacent_index) > 0)
            t_adjacent = floor(s_car_max - s_car(3,1))/(a_adjacent(a_adjacent_index)*delta_t);
        elseif (a_adjacent(a_adjacent_index) < 0)
            t_adjacent = floor(s_car(3,1)-s_car_min)/(-a_adjacent(a_adjacent_index)*delta_t);
        else
            t_adjacent = 1; % dummy case
        end
        a_car(3,1:t_adjacent) = a_adjacent(a_adjacent_index);    

        for V2V = 0:1
            
        % run only once for a_adjacent = 0
            if ((a_adjacent_index == 1) && (V2V ==0))
                continue;
            end
            
            for t = 1:N_t

                if (toc > 60)
                    toc
                    % in the case of non-constant speed, it does not make sure to
                    % keep a sample if the run time exceeds threshold
                    % T_samples(sample) = T_sample_initial_record; in the case
                    % of constant speeds as in ModelOptimization_5.m
                    % or remains 0 in the case of variable speeds
                    disp(['Index = ', num2str(sample), ', T_sample_initial_record = ', num2str(T_sample_initial_record)]);
                    break;
                end

                % see whether the current state is a safe state
                flag = ones(N_cars,1);
                flag(1)=0;
                for i=2:N_cars 

                    if ((N_leading_car > 0) && (i==2))  % leading does exist
                        % leading car
                        d_el = p_car_x(i,t)-p_car_x(1,t);
                        if (d_el < L_car_x)
                            disp('error: d_el < L_car_x');
                        end
                        d_el_tau2 = d_el + (s_car(i,t) - s_car(1,t))*tau/2 - 1/2*a_max*(tau/2)^2;
                        if (d_el_tau2 >= L_car_x)
                            flag(i) = 0;
                        end
                    else
                        % adjacent car
                        d_ei = p_car_x(i,t)-p_car_x(1,t);

                        d_ei_minus_tau2 = d_ei + (s_car(i,t) - s_car(1,t))*tau/2 - 1/2*a_max*(tau/2)^2;
                        d_ei_minus_tau = d_ei + (s_car(i,t) - s_car(1,t))*tau - 1/2*a_max*(tau)^2;
                        if ((d_ei_minus_tau2 >= L_car_x) && (d_ei_minus_tau >= L_car_x))
                            if (d_ei_minus_tau >= max(0,(s_car(1,t)^2-(s_car(i,t)-a_max*tau)^2)/2/a_max)+L_car_x)
                                flag(i) = 0;
                            end
                        end

                        d_ei_plus_tau2 = d_ei + (s_car(i,t) - s_car(1,t))*tau/2 + 1/2*a_max*(tau/2)^2;
                        d_ei_plus_tau = d_ei + (s_car(i,t) - s_car(1,t))*tau + 1/2*a_max*(tau)^2;
                        if ((-d_ei_plus_tau2 >= L_car_x) && (-d_ei_plus_tau >= L_car_x))
                            if (-d_ei_plus_tau >= max(0,(-s_car(1,t)^2+(s_car(i,t)+a_max*tau)^2)/2/a_max)+L_car_x)
                                flag(i) = 0;
                            end
                        end
                    end
                end

                % if the current state is already a safe one, no need to search further
                if (max(flag) == 0)
                    T_samples((a_adjacent_index-1)*2+V2V, sample) = t;
                    disp(['Index = ', num2str(sample), ', T_samples = ', num2str(t)]);
                  %  a_car(1:t)
                    break;
                end

                % if the current state is not a safe one, continue

                % build the table 
                u_car_ego_T = zeros(2*T_max+1,T_max);
                w_car_ego_T = zeros(2*T_max+1,T_max*2);

                k_car_ego_T_min = zeros(1,T_max);
                k_car_ego_T_max = zeros(1,T_max);

                toggle = 1;
                for T=1:T_max

                    n = 0;
                    for k=-T:T

                        % eliminate infeasible terminal speed
                        if (((s_car(1,t)+k*a_max*delta_t)<s_car_min) || ((s_car(1,t)+k*a_max*delta_t)>s_car_max))
                            continue;
                        end

                        n = n+1;
                        if (n==1)
                            k_car_ego_T_min(T) = k;
                        end
                        k_car_ego_T_max(T) = k; 

                        u_car_ego_T(n,T)=k*a_max*delta_t;

                        for m=1:2

                            % calculate relative displacement of ego car at the end of T
                            % m=1: max
                            % m=2: min

                            % for max, T+k
                            % for min, T-k
                            if (m==1)
                                t1 = min(floor((T+k*toggle)/2),floor((s_car_max-s_car(1,t))/a_max/delta_t));
                            else
                                t1 = min(floor((T+k*toggle)/2),floor((-s_car_min+s_car(1,t))/a_max/delta_t));
                            end
                            t2 = T+k*toggle-2*t1;
                            t3 = T-t1-t2;

                            u=0;
                            w_car_ego_T(n,(T-1)*2+m) = 0;
                            for i=1:t1
                                w_car_ego_T(n,(T-1)*2+m) = w_car_ego_T(n,(T-1)*2+m) + u;
                                u = u + toggle;
                            end
                            for i=t1+1:t1+t2
                                w_car_ego_T(n,(T-1)*2+m) = w_car_ego_T(n,(T-1)*2+m) + u;
                            end
                            for i=t1+t2+1:t1+t2+t3
                                w_car_ego_T(n,(T-1)*2+m) = w_car_ego_T(n,(T-1)*2+m) + u;
                                u = u - toggle;
                            end

                            % for max, |a_max| in t1, -|a_max| in t3
                            % for min, -|a_max| in t1, |a_max| in t3
                            % toggle the sign of a_max after every max and min 
                            toggle = - toggle;
                        end
                    end
                end


                % find the smallest T to reach a safe state
                found = 0;

                % 5G V2V correction terms
                ds = zeros(N_cars,T_max);
                dp = zeros(N_cars,T_max);
                if (V2V ~=0)
                    for i=2:N_cars
                        ds(i,1)=a_car(i,t)*delta_t;
                        for T=2:T_max
                            if (t+T-1<=T_max)
                                ds(i,T)=ds(i,T-1)+a_car(i,t+T-1)*delta_t;
                            else
                                ds(i,T)=ds(i,T-1);
                            end
                        end
                        dp(i,1)=0;
                        for T=2:T_max
                            dp(i,T)=dp(i,T-1)+ds(i,T-1)*delta_t;
                        end
                    end
                end
                
                for T=1:T_max
                    for k=k_car_ego_T_min(T):k_car_ego_T_max(T)
                        n = k-k_car_ego_T_min(T)+1;

                        for m=w_car_ego_T(n,(T-1)*2+2):w_car_ego_T(n,(T-1)*2+1) % loop through max and min displacemens

                            flag = ones(N_cars,1);
                            flag(1)=0;

                            for i=2:N_cars 

                                % check whether safe state has been reached

                                % m is an integer. To convert to actual relative
                                % distance, need a factor = a_max*delta_t^2

                                if ((N_leading_car > 0) && (i==2))  % leading does exist
                                    % leading car
                                    d_el = p_car_x(i,t)-p_car_x(1,t) + (s_car(i,t)-s_car(1,t))*T*delta_t - m*a_max*delta_t^2;

                                    d_el = d_el + dp(i,T);

                                    if (d_el >= L_car_x)
                                        d_el_tau2 = d_el + ((s_car(i,t)+ds(i,T)) - (s_car(1,t)+u_car_ego_T(n,T)))*tau/2 - 1/2*a_max*(tau/2)^2;
                                        if (d_el_tau2 >= L_car_x)
                                            flag(i) = 0;
                                        end
                                    end
                                else
                                    % adjacent car
                                    d_ei = p_car_x(i,t)-p_car_x(1,t) + (s_car(i,t)-s_car(1,t))*T*delta_t - m*a_max*delta_t^2;

                                    d_ei = d_ei + dp(i,T);

                                    d_ei_minus_tau2 = d_ei + ((s_car(i,t)+ds(i,T)) - (s_car(1,t)+u_car_ego_T(n,T)))*tau/2 - 1/2*a_max*(tau/2)^2;
                                    d_ei_minus_tau = d_ei + ((s_car(i,t)+ds(i,T)) - (s_car(1,t)+u_car_ego_T(n,T)))*tau - 1/2*a_max*(tau)^2;
                                    if ((d_ei_minus_tau2 >= L_car_x) && (d_ei_minus_tau >= L_car_x))
                                        if (d_ei_minus_tau >= max(0,((s_car(1,t)+u_car_ego_T(n,T))^2-(s_car(i,t)+ds(i,T)-a_max*tau)^2)/2/a_max)+L_car_x)
                                            flag(i) = 0;
                                        end
                                    end

                                    d_ei_plus_tau2 = d_ei + ((s_car(i,t)+ds(i,T)) - (s_car(1,t)+u_car_ego_T(n,T)))*tau/2 + 1/2*a_max*(tau/2)^2;
                                    d_ei_plus_tau = d_ei + ((s_car(i,t)+ds(i,T)) - (s_car(1,t)+u_car_ego_T(n,T)))*tau + 1/2*a_max*(tau)^2;
                                    if ((-d_ei_plus_tau2 >= L_car_x) && (-d_ei_plus_tau >= L_car_x))
                                        if (-d_ei_plus_tau >= max(0,(-(s_car(1,t)+u_car_ego_T(n,T))^2+(s_car(i,t)+ds(i,T)+a_max*tau)^2)/2/a_max)+L_car_x)
                                            flag(i) = 0;
                                        end
                                    end
                                end
                            end
                            if (max(flag) == 0) % if one between max and min (inclusive) works, then already find a solution

                                T_found = T;
                                k_found = k;
                                m_found = m;

                                % start from w_min, find the specific path that satisfies T_found,
                                % k_found, m_found
                                t1 = min(floor((T_found-k_found)/2),floor((-s_car_min+s_car(1,t))/a_max/delta_t));
                                t2 = T_found-k_found-2*t1;
                                t3 = T_found-t1-t2;

                                u=zeros(T_found,1);
                                for i=2:t1+1
                                    u(i) = u(i-1) - 1;
                                end
                                for i=t1+2:t1+t2+1
                                    u(i) = u(i-1);
                                end
                                for i=t1+t2+2:t1+t2+t3
                                    u(i) = u(i-1) + 1;
                                end

                                fixed = zeros(T_found,1);
                                fixed(1) = 1;

                                v=u;

                                if (v(T_found)-k_found>0)
                                    fixed(T_found) = 1;
                                end
                                for i=2:T_found-1
                                    if ((v(i)>v(i-1)) && (fixed(i-1) ==1))
                                        fixed(i) = 1;
                                    end
                                    if ((v(i)>v(i+1)) && (fixed(i+1) ==1))
                                        fixed(i) = 1;
                                    end
                                end

                                while sum(v)<m_found
                                    w = v;
                                    for i=1:T_found
                                        if (fixed(i) == 1)
                                            w(i) = T_max; % some very positive number
                                        end
                                    end   
                                    I = find(w==min(w),1,'last');
                                    v(I) = v(I)+1;

                                    if (v(T_found)-k_found>0)
                                        fixed(T_found) = 1;
                                    end
                                    for i=2:T_found-1
                                        if ((v(i)>v(i-1)) && (fixed(i-1) ==1))
                                            fixed(i) = 1;
                                        end
                                        if ((v(i)>v(i+1)) && (fixed(i+1) ==1))
                                            fixed(i) = 1;
                                        end
                                    end
                                end

                                a_ego = zeros(T_found,1);
                                for n=1:T_found-1
                                    a_ego(n) = v(n+1)-v(n);
                                end
                                a_ego(T_found) = k_found - v(T_found);
                                a_ego = a_ego * a_max;

                                % if there is a leading car, then have to check each
                                % time instant to see whether safe distance condition
                                % is met
                                flag = 0;
                                if (N_leading_car > 0)

                                    p_car_x_ego = p_car_x(1,t);
                                    p_car_x_leading = p_car_x(2,t);
                                    s_car_x_ego = s_car(1,t);
                                    s_car_x_leading = s_car(2,t);


                                    for tt = 1:T_found
                                        p_car_x_ego = p_car_x_ego + s_car_x_ego*delta_t;
                                        s_car_x_ego = s_car_x_ego + a_ego(tt)*delta_t;
                                        p_car_x_leading = p_car_x_leading + s_car_x_leading*delta_t + dp(2,tt);

                                        if (p_car_x_leading - p_car_x_ego < max(0,(s_car_x_ego^2-(s_car_x_leading+ds(2,tt))^2)/2/a_max)+L_car_x)
                                            flag = 1; 
                                            break;
                                        end
                                    end
                                end

                                if (flag ==0)
                                    found = 1;
                                    break;
                                end
                            end
                        end
                        if (found == 1)
                            break;
                        end
                    end
                    if (found == 1)
                        break;
                    end        
                end

                if (found == 0)
                    T_samples((a_adjacent_index-1)*2+V2V, sample) = T_max;
                    disp(['Index = ', num2str(sample), 'cannot find a solution']);
                    break;
                end

                if (t == 1)
                    T_sample_initial_record = length(a_ego) + 1;
                end

                % update positions and speeds of all cars
                p_car_x(:,t+1) = p_car_x(:,t) + s_car(:,t)*delta_t; 

                % apply the first a_car 
                % first check whether a_ego(1) will lead to unsafe distance from the
                % leading car
                a_ego_max = a_max;
                if (N_leading_car > 0)

                    % for the worst case of potential collision, one should use
                    %s_car_leading = s_car(2,t) - a_max*delta_t;
                    % however, the algorithm assumes constant speed of the leading car
                    % if we add -a_max*delta_t here, the algorithm will not be
                    % consistent, meaning that an optimal solution obtained from the
                    % above code will be capped here. This inconsistency leads to much
                    % longer T.
                    % For simplicity, use the following code instead
                    %s_car_leading = s_car(2,t) - a_max*delta_t*0;
                    s_car_leading = s_car(2,t) + ds(2,1);

                    d_el = p_car_x(2,t+1)-p_car_x(1,t+1) + dp(2,1);
                    if (d_el >= max(0,((s_car(1,t)+a_max*delta_t)^2-s_car_leading^2)/2/a_max)+L_car_x)
                        a_ego_max = a_max;
                    elseif (d_el >= max(0,((s_car(1,t))^2-s_car_leading^2)/2/a_max)+L_car_x)
                        a_ego_max = 0;
                    else
                        a_ego_max = -a_max;
                    end
                end
                a_car(1,t) = min(a_ego(1), a_ego_max);
            %    if (a_ego(1) ~= a_ego_max)
            %        disp('not equal!!');
            %    end

                s_car(:,t+1) = s_car(:,t) + a_car(:,t)*delta_t;
            end
            
        end
        
    end
    
end

% variable constant speed
save('ModelOptimization_7.mat','T_samples')




