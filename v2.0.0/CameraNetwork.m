function [xyCams, th0, ang_vis] = CameraNetwork(row_Cam, col_Cam, MX, MY, R, obstacle, so_x, so_y, po_x, po_y, nCams)

    th0 = zeros(1,nCams);
    ang_vis = zeros(1,nCams);
    
    if obstacle == 0
        l = linspace(1,MX-1,row_Cam);
        b = linspace(1,MY-1,col_Cam);
        [CAMX,CAMY] = meshgrid(l,b);
        xyCams = zeros(nCams,2);
        camx_vect = zeros(nCams,1);
        camy_vect = zeros(nCams,1);
        
        if col_Cam~=row_Cam
            CAMZ = CAMX;
            CAMX = CAMY';
            CAMY = CAMZ';
        end

        c = 1;
        for i = 1:col_Cam
            for j = 1:row_Cam
                camx_vect(c) = CAMX(i,j);
                camy_vect(c) = CAMY(i,j);
                xyCams(c, :) = [CAMX(i,j) CAMY(i,j)];
                c = c + 1;
            end
        end
        
        figure;
        hold on;
        grid on;
        xlim([-10, MX+10])
        ylim([-10, MY+10])
        plot(xyCams(:,1), xyCams(:,2),'o','MarkerSize',10,'MarkerFaceColor','g');
        plot([0;MX], [0, 0],'k','LineWidth',2); 
        plot([0;MX], [MY, MY],'k','LineWidth',2); 
        plot([0;0], [0, MY],'k','LineWidth',2); 
        plot([MX;MX], [0, MY],'k','LineWidth',2);
        
        c = 0;
        for i = 1:row_Cam
            for j = 1:col_Cam

                c = c+1;
                th0(c) = 0;
                ang_vis(c) = pi/2;
                bool = 0;
                %Camera on the top-right
                if c == nCams
                    th0(c) = pi;
                    ang_vis(c) = 3*pi/2;
                    bool = 1;
                end
                %Camera on the down-left
                if c == 1
                    th0(c) = 0;
                    ang_vis(c) = pi/2;
                    bool = 1;

                end
                %Camera on the down-right
                if j == col_Cam && i == 1
                    th0(c) = pi/2;
                    ang_vis(c) = pi;
                    bool = 1;
                end
                %Camera on the up-left
                if i == row_Cam && j == 1
                    th0(c) = 3*pi/2;
                    ang_vis(c) = 2*pi;
                    bool = 1;
                end
                % Other camera
                if bool == 0
                    if j == 1
                       th0(c) = -pi/4;
                       ang_vis(c) = pi/4;
                    end
                    if j == col_Cam
                       th0(c) = 3*pi/4;
                       ang_vis(c) = 5*pi/4;
                    end
                    if i == 1
                        th0(c) = pi/4;
                        ang_vis(c) = 3*pi/4;
                    end
                    if i == row_Cam
                        th0(c) = 5*pi/4;
                        ang_vis(c) = 7*pi/4;
                    end
                end

                th = th0(c):pi/50:ang_vis(c);
                xunit = R * cos(th) + xyCams(c,1);
                yunit = R * sin(th) + xyCams(c,2);

                plot(xunit, yunit,'b','LineWidth',1);
                plot([xyCams(c,1); R * cos(ang_vis(c)) + xyCams(c,1)], [xyCams(c,2), R * sin(ang_vis(c)) + xyCams(c,2)],'b','LineWidth',1);
                plot([xyCams(c,1); R * cos(th0(c)) + xyCams(c,1)], [xyCams(c,2), R * sin(th0(c)) + xyCams(c,2)],'b','LineWidth',1);
                
            end
        end
        
        
    else
        
        l = linspace(1,MY-1,row_Cam); 
        b = linspace(1,MX-1,col_Cam);
        [CAMX,CAMY] = meshgrid(b,l);

        xyCams = zeros(nCams,2);
        camx_vect = zeros(nCams,1);
        camy_vect = zeros(nCams,1);

        c = 1;
        for i = 1:row_Cam
            for j = 1:col_Cam
                camx_vect(c) = CAMX(i,j);
                camy_vect(c) = CAMY(i,j);
                xyCams(c, :) = [CAMX(i,j) CAMY(i,j)];
                c = c + 1;
            end
        end
     
        %Check if the XYCAMS are inside of the arena
               
        r=1; %starting point
        nCams_inside = nCams - 2*(row_Cam + col_Cam -2); %number cameras inside
        space = 2*(so_x+so_y) / nCams_inside;
        for c = 1:nCams
            if camx_vect(c) > 1 && camx_vect(c) < MX-1 && camy_vect(c) > 1 && camy_vect(c) < MY-1
                
                if r<so_x
                    xyCams(c,:) = [(po_x + r), po_y]; %Bottom side
                    th0(c) = 5*pi/4;
                    ang_vis(c) = 7*pi/4;
                elseif r<so_x+so_y
                    xyCams(c,:) = [(po_x + so_x), (po_y + r - so_x)]; %Right side
                    th0(c) = -pi/4;
                    ang_vis(c) = pi/4;                   
                elseif r<2*so_x+so_y
                    xyCams(c,:) = [(po_x+so_x+(so_x+so_y-r)), (po_y + so_y)]; %Upper side
                    th0(c) = pi/4;
                    ang_vis(c) = 3*pi/4;
                else
                    xyCams(c,:) = [po_x, (po_y + so_y +(2*so_x + so_y -r))]; %Left side
                    th0(c) = 3*pi/4;
                    ang_vis(c) = 5*pi/4;
                end
                r = r + space;
                
            end
        end
        
        figure;
        hold on;
        grid on;
        xlim([-10,MX+10])
        ylim([-10,MY+10])
        hold on;
        rectangle('Position', [po_x,po_y,so_x,so_y], 'FaceColor', 'k')
        hold on;
        plot(xyCams(:,1), xyCams(:,2),'o','MarkerSize',10,'MarkerFaceColor','g');
        hold on;
        plot([0;MX], [0, 0],'k','LineWidth',2); 
        plot([0;MX], [MY, MY],'k','LineWidth',2); 
        plot([0;0], [0, MY],'k','LineWidth',2); 
        plot([MX;MX], [0, MY],'k','LineWidth',2);
 
        c = 0;
        for i = 1:row_Cam
            for j = 1:col_Cam
                c = c+1;   
                bool = 0;
                %Camera on the top-right                    
                if i == row_Cam && j == col_Cam 
                    th0(c) = pi;
                    ang_vis(c) = 3*pi/2;
                    bool = 1;
                end
                %Camera on the down-left
                if c == 1
                    th0(c) = 0;
                    ang_vis(c) = pi/2;
                    bool = 1;
                end
                %Camera on the down-right
                if j == col_Cam && i == 1
                    th0(c) = pi/2;
                    ang_vis(c) = pi;
                    bool = 1;
                end
                %Camera on the up-left
                if i == row_Cam && j == 1
                    th0(c) = 3*pi/2;
                    ang_vis(c) = 2*pi;
                    bool = 1;
                end
                % Other camera on the external side
                if bool == 0
                    if j == 1
                       th0(c) = -pi/4;
                       ang_vis(c) = pi/4;
                    end
                    if j == col_Cam
                       th0(c) = 3*pi/4;
                       ang_vis(c) = 5*pi/4;
                    end
                    if i == 1
                        th0(c) = pi/4;
                        ang_vis(c) = 3*pi/4;
                    end
                    if i == row_Cam
                        th0(c) = 5*pi/4;
                        ang_vis(c) = 7*pi/4;
                    end
                end
            end
        end
        for c = 1: nCams
            th = th0(c):pi/50:ang_vis(c);
            xunit = R * cos(th) + xyCams(c,1);
            yunit = R * sin(th) + xyCams(c,2);

            plot(xunit, yunit,'b','LineWidth',1);
            plot([xyCams(c,1); R * cos(ang_vis(c)) + xyCams(c,1)], [xyCams(c,2), R * sin(ang_vis(c)) + xyCams(c,2)],'b','LineWidth',1);
            plot([xyCams(c,1); R * cos(th0(c)) + xyCams(c,1)], [xyCams(c,2), R * sin(th0(c)) + xyCams(c,2)],'b','LineWidth',1);
        end
    end
end