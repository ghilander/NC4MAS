function xk = RW(xk_past, F, G, params, obstacle, size_obs_x, size_obs_y, pos_obs_x, pos_obs_y)
    % xkm1(1) = coord. x
    % xkm1(2) = coord. y
    % xkm1(3) = speed x
    % xkm1(4) = speed y

    Mx = params(1);
    My = params(2);
    
    %Model
    xk = F*xk_past'+G*randn(2,1);
    if obstacle == 0
        %Check the limits
        if xk(1)>Mx
            xk = xk_past;
            xk(3:4) = -xk(3:4);
        end
        if xk(2)>My
            xk = xk_past;
            xk(3:4) = -xk(3:4);
        end
        if xk(1)<0
            xk = xk_past;
            xk(3:4) = -xk(3:4);
        end
        if xk(2)<0
            xk = xk_past;
            xk(3:4) = -xk(3:4);
        end
    else
        %Check the limits
        if xk(1)>Mx
            xk = xk_past;
            xk(3:4) = -xk(3:4);
        end
        if xk(2)>My
            xk = xk_past;
            xk(3:4) = -xk(3:4);
        end
        if xk(1)<0
            xk = xk_past;
            xk(3:4) = -xk(3:4);
        end
        if xk(2)<0
            xk = xk_past;
            xk(3:4) = -xk(3:4);
        end
        
        %Check the obstacle
        if xk(1)>pos_obs_x && xk(1)<pos_obs_x+size_obs_x && xk(2)>pos_obs_y && xk(2)<pos_obs_y+size_obs_y
            xk = xk_past;
            xk(3:4) = -xk(3:4);
        end
    end
end