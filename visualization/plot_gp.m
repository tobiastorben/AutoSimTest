function plot_gp(parameters, rho_mean, rho_var, rho_obs, P, P_obs, fig_handle)
    figure(fig_handle);
    if length(parameters) == 1%1D Plot
        clf
        lower = rho_mean - 3*sqrt(rho_var);
        upper = rho_mean + 3*sqrt(rho_var);
        conf = [lower; flipud(upper)];
        p_conf = [P'; flipud(P')];
        plot(P, rho_mean, '-k', 'LineWidth', 1);
        hold on;
        h = fill(p_conf, conf,[0.5 0.5 0.5], 'LineStyle','none');
        set(h,'facealpha',.2)
        plot(P_obs, rho_obs, 'ro')
        legend('GP Mean', '99% confidence interval',  'Observed points');
        ylabel('Robustness')
        xlabel(parameters(1).Name);
        grid on;
        drawnow;
    elseif length(parameters) == 2%Surface plot
        clf
        [X,Y] = meshgrid(parameters(2).Values,parameters(1).Values);
        Z = reshape(rho_mean, size(X));
        %Z = min(Z,1);Z=max(Z,-1);
        plot3(P_obs(2,:), P_obs(1,:), rho_obs, 'o', 'MarkerFaceColor', 'k');
        hold on;
        surfc(X,Y,Z)
        xlabel(parameters(2).Name);
        ylabel(parameters(1).Name);
        zlabel('Robustness');
        legend('Observations', 'GP Mean');
        grid on;
        drawnow;
        
    else %Visualization not supported for higher dimensional sets
        return
    end

end