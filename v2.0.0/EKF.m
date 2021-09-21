function [Xhat, Phat] = EKF(F, G, ED, V, Y, cvX0, mP0, R, Q, beta)

    % number active camera
    
    n = length(Y);
    
    % Prediction step
    Xpred = F*cvX0;
    Ppred = F*mP0*F' + G*Q*G';
       
    % Evaluate the euclidian distance with x(k|k-1)    
    Ybar = zeros(n,1);
    H = zeros(n,4);
    H2 = zeros(n,1);
    
    for i=1:n
        distance = ED{i};
        % Compute the jacobian h() w.r.t V for all active camera 
        h1 = jacobian(distance, V);
        H(i,:) = double(subs(h1, V, cvX0)); % evaluate jacob. with x(k|k-1)
        H2(i) = distance(cvX0(1), cvX0(2), cvX0(3), cvX0(4));
        Ybar(i) = Y(i) - H2(i) + H(i,:)*cvX0;
    end
    
    % Filtered step
    
    Phat = inv(inv(Ppred) + H'*inv(R)*H);
    
    Xhat = Xpred + Phat*H'*inv(R)*beta*(Ybar - H*Xpred); %weighted prediction
    
end