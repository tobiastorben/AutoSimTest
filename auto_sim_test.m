function [retval, rho_mean, rho_var, rho_obs, P, P_obs, fig_handle] = auto_sim_test(sim_fun, parameters, hypers, stl_formula, predicates, plot_file)


%% Recording
writerObj = VideoWriter(plot_file);
writerObj.FrameRate = 5;
open(writerObj);
i_frame = 1;

%% Initialization
n = 1;
n_obs = 0;
k = length(parameters);
for i = 1:k
    n = n*length(parameters(i).Values);
end
idx = ones(k);
P_obs = [];
rho_min_conf = -Inf;
rho_obs = [];
rho_mean = zeros(n,1);
rho_var = hypers.sigma^2*ones(n,1);

fig_handle = figure();

%% Generate list of points P
P = zeros(k,n);
for i = 1:n
    if i > 1
        idx = update_idx(parameters, idx, 1);
    end
    p = [];
    for j = 1:k
        p = [p; parameters(j).Values(idx(j))];
    end
    P(:,i) = p;
end

%% Plot prior
if k == 1
    plot_gp(parameters, rho_mean, rho_var, rho_obs, P, P_obs, fig_handle);
    F(i_frame) = getframe(gcf);
    writeVideo(writerObj, F(i_frame));
    i_frame = i_frame + 1;
end
%% Find the robustness of hypers.n_seed seed points
%Draw seed points using Latin Hypercupe Sampling (LHS)
lhs = lhsdesign(hypers.n_seed, k)';
P_seed = zeros(k, hypers.n_seed);
observed = zeros(n,1);
for j = 1:k
    lower = parameters(j).Values(1);
    upper = parameters(j).Values(end);
    P_seed(j,:) = lower + (upper-lower)*lhs(j,:);
end

for j = 1:hypers.n_seed
    %Find closest point in P
    p_seed = P_seed(:,j);
    for k = 1:n
        err(k) = norm(P(:,k)-p_seed); 
    end
    [~,ind] = min(err);
    p = P(:,ind);
    %Run simulation
    [w,t] = sim_fun(p);
    %Find robustness
    rho = dp_taliro(stl_formula,predicates,w,t);
    observed(ind) = 1;
    rho_obs = [rho_obs; rho];
    P_obs = [P_obs P_seed(:,j)];
    n_obs = n_obs + 1;
end

%% Run new simulations until Verified or Falsified
for iter = hypers.n_seed+1:hypers.maxNumberOfSimulations
    %Build Gaussian Process based on observation so far
    gp = fitrgp(P_obs', rho_obs,'KernelFunction', hypers.kernelfun,'Sigma',hypers.sigma_n, 'KernelParameters',[hypers.length_scales;hypers.sigma], 'Verbose', 0, 'FitMethod', 'none');
    [rho_mean,~,confidence_bounds] = predict(gp, P');
    rho_var = (0.5*(confidence_bounds(:,2) - rho_mean)).^2;
    
    %Plot results
    plot_gp(parameters, rho_mean, rho_var, rho_obs, P, P_obs, fig_handle);
    F(i_frame) = getframe(gcf);
    writeVideo(writerObj, F(i_frame));
    i_frame = i_frame + 1;
      
    %Find the point with lowest confidence
    lower_confidence_bound = rho_mean - hypers.n_conf*sqrt(rho_var);
    rho_min_conf = min(lower_confidence_bound);
    
    %Terminate if verified
    if rho_min_conf > 0.0
        retval = 'Verified'
        close(writerObj);
        return
    end
    
    %Terminate if Falsified
    if min(rho_obs) < 0.0
        retval = 'Falsified'
        close(writerObj);
        return
    end
    
    %Find next test point
    lower_confidence_bound = rho_mean - hypers.kappa*sqrt(rho_var);
    for i = 1:n
       if observed(i) == 1
          lower_confidence_bound(i) = inf; 
       end
    end
    [~, min_idx] = min(lower_confidence_bound);
    p = P(:, min_idx(1));
    %Run simulation
    [w,t] = sim_fun(p);
    %Find robustness
    rho = dp_taliro(stl_formula,predicates,w,t);
    rho_obs = [rho_obs; rho];
    observed(min_idx(1)) = 1;
    P_obs = [P_obs p];
    n_obs = n_obs + 1;
end
retval = 'Inconclusive'
%Plot results
plot_gp(parameters, rho_mean, rho_var, rho_obs, P, P_obs);
F(i_frame) = getframe(gcf);
writeVideo(writerObj, F(i_frame));
i_frame = i_frame + 1;
close(writerObj);
return
end

function idx = update_idx(parameters, idx, j)
idx(j) = idx(j) + 1;
if idx(j) > length(parameters(j).Values)
    idx(j) = 1;
    idx = update_idx(parameters, idx, j+1);
end
end