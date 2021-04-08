function rho = online_robustness(phi, pred, w, t)
    for i = 1:length(t)
       rho(i) =  dp_taliro(phi,pred,w(:,i:end)',t(i:end)');
    end
end