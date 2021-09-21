function [camera_activated, dist, theta_rad] = target(xk, yk, dk, th0, ang_vis, nTarget, XYCAMS, nCams, radius,v)

    camera_activated = zeros(nCams,nTarget);
    for c = 1:nCams
        th = th0(c):pi/50:ang_vis(c);
        xunit = radius * cos(th) + XYCAMS(c,1);
        yunit = radius * sin(th) + XYCAMS(c,2);

        %CHECK IF A CAMERA IS ACTIVED
        %Evaluation of Euclidean distance
        for k = 1:nTarget
            %Evaluation of distance
            dist(c,k) = sqrt((xk(:,:,k)-XYCAMS(c,1))^2+(yk(:,:,k)-XYCAMS(c,2))^2);
            %Evaluate if the camera can see the target
            theta_rad(c,k) = atan2((yk(:,:,k)-XYCAMS(c,2)),(xk(:,:,k)-XYCAMS(c,1)));
            if theta_rad(c,k)<0
                theta_rad(c,k) = theta_rad(c,k)+2*pi;
            end                
        end
        for k = 1:nTarget
            if theta_rad(c,k)>7*pi/4 && th0(c) <= abs(-pi/4+eps) && ang_vis(c) <= abs(pi/4+eps)
                theta_rad(c,k) = theta_rad(c,k)-2*pi;
            end
            if dist(c,k)<radius && dk(k)==1 && theta_rad(c,k) <= ang_vis(c) && theta_rad(c,k) >= th0(c) %check if camera sees
                camera_activated(c,k) = 1;
            end
            plot(xk(:,:,k),yk(:,:,k),'ok','MarkerSize',5,'MarkerFaceColor',v(k,:))   %plot target          
        end
        if any(camera_activated(c,:)) %if camera see at least one target
            plot(xunit, yunit,'r','LineWidth',1);
            plot([XYCAMS(c,1); radius * cos(ang_vis(c)) + XYCAMS(c,1)], [XYCAMS(c,2), radius * sin(ang_vis(c)) + XYCAMS(c,2)],'r','LineWidth',1);
            plot([XYCAMS(c,1); radius * cos(th0(c)) + XYCAMS(c,1)], [XYCAMS(c,2), radius * sin(th0(c)) + XYCAMS(c,2)],'r','LineWidth',1);
        else
            plot(xunit, yunit,'b','LineWidth',1);
            plot([XYCAMS(c,1); radius * cos(ang_vis(c)) + XYCAMS(c,1)], [XYCAMS(c,2), radius * sin(ang_vis(c)) + XYCAMS(c,2)],'b','LineWidth',1);
            plot([XYCAMS(c,1); radius * cos(th0(c)) + XYCAMS(c,1)], [XYCAMS(c,2), radius * sin(th0(c)) + XYCAMS(c,2)],'b','LineWidth',1);
        end
        
    end
end