function [beta, MP] = Bernoulli_estimation(beta0, beta_MP0, X, xyCams, distances, camera_activates)
    
    nTarget = size(camera_activates,2);
    beta_cam_activated = beta0(find(camera_activates)); 
    
    % Possiamo calcolare la posizione predetta 
end