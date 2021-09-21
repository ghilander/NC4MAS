close all
clear all
clc

%% TO DO

% aggiungere attivazione e raggio visione camere nell'ultimo plot

% aggiungere rumore 

%% Setting up initial conditions
nTarget = 3;                %Number of target
%dk = randi([0, 1], [1, nTarget]); %health of target
%dk = ones(1,nTarget);
dk = [1,0,1];
target_sick = sum(dk); %number of sick targets
sick_pos = find(dk);
%color map
v = zeros(nTarget,3);
for i =1:nTarget
    if dk(i)
        v(i,:) = [1,rand(1)*0.5,rand(1)*0.5]; %sick
    else
        v(i,:) = [rand(1)*0.5,rand(1)*0.5,1];
    end
end
Mx = 50;                    %Limit of axis x
My = 50;                    %Limit of axis y
size_room = [Mx, My];
radius = 20;


%Matrix of camera
row_Cam = 4;                %Number of cams in each column
col_Cam = 5;                %Number of cams in each row
nCams = row_Cam*col_Cam;

%Plot of the camera network
obstacle = 1;               %obstacle = 1, no obstacle = 0
size_obs_x = Mx/5;          %maintain the proportional
size_obs_y = My/5;
pos_obs_x = (Mx-size_obs_x)/2;
pos_obs_y = (My-size_obs_y)/2;

%Inital position target
X0 = 50*rand(1,nTarget);
Y0 = 50*rand(1,nTarget);
Vx0 = 0.01*ones(1,nTarget);
Vy0 = 0.01*ones(1,nTarget);
if obstacle == 1
    %set initial position not in the obstacle area
    for i =1:nTarget
        while X0(i)>pos_obs_x && X0(i)<pos_obs_x+size_obs_x && Y0(i)>pos_obs_y && Y0(i)<pos_obs_y+size_obs_y
            X0(i) = 50*rand(1);
            Y0(i) = 50*rand(1);
        end
    end
end
Ts = 1;
x0 = [X0; Y0; Vx0; Vy0]; %4*nTarget state matrix

%Setting up of the model
F = [eye(2) 0.5*Ts*eye(2);
     zeros(2) eye(2)];
G = [0.3*Ts^2*eye(2) 0.1*Ts*eye(2)]';
%G = [0.5*Ts^2*eye(2) Ts*eye(2)]';

%Covariance matrix
Q = 0.01*eye(2);
r = 0.1*eye(1);             % j-th sensor noise 
Rrep = repmat(r, 1, nCams);
Rcell = mat2cell(Rrep, size(r,1), repmat(size(r, 2), 1, nCams)); % Create Cell Array Of Orignal Repeated Matrix
R = blkdiag(Rcell{:});      % Covariance matrix for all sensors


% Define the arena
[XYCAMS, th0, ang_vis] = CameraNetwork(row_Cam, col_Cam, Mx, My, radius, obstacle, size_obs_x, size_obs_y, pos_obs_x, pos_obs_y, nCams);

%% Initialize the Random Walk model
%Number of step
N = 100;
X = zeros(N,4,nTarget);     %State vector
X(1,:,:) = x0;
for i = 1:nTarget
    
    for k = 2:N
        X(k,:,i) = RW(X(k-1,:,i), F, G, size_room, obstacle, size_obs_x, size_obs_y, pos_obs_x, pos_obs_y);
    end
end

% Graph plot
for i = 1:nTarget
    
    figure(2)
    subplot(nTarget,1,i)
    hold on;
    plot(0:N-1,X(:,1,i));
    plot(0:N-1,X(:,2,i),'r');
    hold off;
    legend('pos_{x}', 'pos_{y}'); 
end
%Maps target plot
figure(3)
hold on;
grid on;
axis([-10 Mx+10 -10 My+10])
hold on;
plot([0;Mx], [0, 0],'k','LineWidth',2); 
plot([0;Mx], [My, My],'k','LineWidth',2); 
plot([0;0], [0, My],'k','LineWidth',2); 
plot([Mx;Mx], [0, My],'k','LineWidth',2);
hold on;
rectangle('Position', [pos_obs_x,pos_obs_y,size_obs_x,size_obs_y], 'FaceColor', 'k')
hold on;

for i = 1:nTarget
    plot(X(:,1,i),X(:,2,i),'ok','MarkerSize',5,'MarkerFaceColor',v(i,:));
end


%% Verify in the random walk which camera is active 'real time'

figure(4);
hold on;
grid on;
axis([-10 Mx+10 -10 My+10])
hold on;
plot([0;Mx], [0, 0],'k','LineWidth',2); 
plot([0;Mx], [My, My],'k','LineWidth',2); 
plot([0;0], [0, My],'k','LineWidth',2); 
plot([Mx;Mx], [0, My],'k','LineWidth',2);
hold on;
rectangle('Position', [pos_obs_x,pos_obs_y,size_obs_x,size_obs_y], 'FaceColor', 'k')
hold on;
plot(XYCAMS(:,1), XYCAMS(:,2),'o','MarkerSize',10,'MarkerFaceColor','g');
hold on;

camera_activated_extended = zeros(N, nCams, nTarget);
dist_extended = zeros(N, nCams, nTarget);
theta_rad_extended = zeros(N, nCams, nTarget);
    
for k = 1:N
    %Calculate for each instant which cameras are activated and the distances 
    [camera_activated, dist, theta_rad] = target(X(k,1,:), X(k,2,:), dk, th0, ang_vis, nTarget, XYCAMS, nCams, radius,v);
    pause(.1)
    camera_activated_extended(k,:,:) = camera_activated;
    dist_extended(k,:,:) = dist;
    theta_rad_extended(k,:,:) = theta_rad;

end


%% Setup Extended Kalman Filter

cvXhat = [];

syms  z1 z2 z3 z4;
V = [z1; z2; z3; z4];

% Initialization:
cvX0 = zeros(4,target_sick);
mP0 = zeros(4,4,target_sick);
for i =1:target_sick
    mP0(:,:,i) = 100*eye(4);
end
% Bernouilli vector
B = ones(nCams,1); %all cameras can see well

x_vect = zeros(4,N,target_sick);

for k = 1:N %for each step 
    
    for i = 1 :target_sick      % only for sick target compute the estimation
        if dk(sick_pos(i)) 
            ED = {};            % Euclidean Distance
            indexes = [];       % misure SOLO delle telecamere che vedono il target
            c = 1;

            for j = 1:nCams
                % Prepare the function to calculate the jacobian
                if camera_activated_extended(k,j,sick_pos(i))       % check if camera can see                                
                    ED{c} = @(z1, z2, z3, z4) sqrt((XYCAMS(j,1) - z1)^2 + (XYCAMS(j,2) - z2)^2 );
                    c = c + 1;
                end
            end
            % All measurements of cameras that can see the target
            Yk = dist_extended(k,(find(camera_activated_extended(k,:,sick_pos(i)))), sick_pos(i)); 
            beta = diag(B(find(camera_activated_extended(k,:,sick_pos(i))))); %bernoulli parameters
            Rrep = repmat(r, 1, length(Yk));
            Rcell = mat2cell(Rrep, size(r,1), repmat(size(r, 2), 1, length(Yk)));
            R = blkdiag(Rcell{:}); % Covariance matrix for all sensors

            % Kalman Filter step
            [cvXhat, mP] = EKF(F, G, ED, V, Yk, cvX0(:,i), mP0(:,:,i), R, Q, beta);

            % save the position
            x_vect(:,k,i) = cvXhat; 
            % uptade the intial conditions
            
            cvX0(:,i) = cvXhat;
            mP0(:,:,i) = mP;
        end
    end
end

% Grafico i risultati:
figure(5);
hold on;
grid on;
axis([-10 Mx+10 -10 My+10])
hold on;
plot([0;Mx], [0, 0],'k','LineWidth',2); 
plot([0;Mx], [My, My],'k','LineWidth',2); 
plot([0;0], [0, My],'k','LineWidth',2); 
plot([Mx;Mx], [0, My],'k','LineWidth',2);
hold on;
rectangle('Position', [pos_obs_x,pos_obs_y,size_obs_x,size_obs_y], 'FaceColor', 'k')
hold on;
for k =1 : N   
    cc = 1;
    for i = 1:nTarget
        plot(X(k,1,i),X(k,2,i),'ok','MarkerSize',5,'MarkerFaceColor',v(i,:));
        if dk(i)
            plot(x_vect(1, k,cc), x_vect(2, k,cc),'ok','MarkerSize',5,'MarkerFaceColor', 'y');
            cc = cc +1;
        end
    end
    pause(.1);
end

%% EKF with fixed Bernoulli 

cvXhat = [];

syms  z1 z2 z3 z4;
V = [z1; z2; z3; z4];

% Initialization:
cvX0 = zeros(4,target_sick);
mP0 = zeros(4,4,target_sick);
for i =1:target_sick
    mP0(:,:,i) = 100*eye(4);
end
% Bernouilli vector
B = ones(nCams,1); %all cameras can see well

B(1:2:end) = .3; %half cameras can see well 

x_vect = zeros(4,N,target_sick);

for k = 1:N %for each step 
    
    for i = 1 :target_sick      % only for sick target compute the estimation
        if dk(sick_pos(i)) 
            ED = {};            % Euclidean Distance
            indexes = [];       % misure SOLO delle telecamere che vedono il target
            c = 1;

            for j = 1:nCams
                % Prepare the function to calculate the jacobian
                if camera_activated_extended(k,j,sick_pos(i))       % check if camera can see                                
                    ED{c} = @(z1, z2, z3, z4) sqrt((XYCAMS(j,1) - z1)^2 + (XYCAMS(j,2) - z2)^2 );
                    c = c + 1;
                end
            end
            % All measurements of cameras that can see the target
            Yk = dist_extended(k,(find(camera_activated_extended(k,:,sick_pos(i)))), sick_pos(i)); 
            beta = diag(B(find(camera_activated_extended(k,:,sick_pos(i))))); %bernoulli parameters
            Rrep = repmat(r, 1, length(Yk));
            Rcell = mat2cell(Rrep, size(r,1), repmat(size(r, 2), 1, length(Yk)));
            R = blkdiag(Rcell{:}); % Covariance matrix for all sensors

            % Kalman Filter step
            [cvXhat, mP] = EKF(F, G, ED, V, Yk, cvX0(:,i), mP0(:,:,i), R, Q, beta);

            % save the position
            x_vect(:,k,i) = cvXhat; 
            % uptade the intial conditions
            
            cvX0(:,i) = cvXhat;
            mP0(:,:,i) = mP;
        end
    end
end

% Grafico i risultati:
figure(5);
hold on;
grid on;
axis([-10 Mx+10 -10 My+10])
hold on;
plot([0;Mx], [0, 0],'k','LineWidth',2); 
plot([0;Mx], [My, My],'k','LineWidth',2); 
plot([0;0], [0, My],'k','LineWidth',2); 
plot([Mx;Mx], [0, My],'k','LineWidth',2);
hold on;
rectangle('Position', [pos_obs_x,pos_obs_y,size_obs_x,size_obs_y], 'FaceColor', 'k')
hold on;
for k =1 : N   
    cc = 1;
    for i = 1:nTarget
        plot(X(k,1,i),X(k,2,i),'ok','MarkerSize',5,'MarkerFaceColor',v(i,:));
        if dk(i)
            plot(x_vect(1, k,cc), x_vect(2, k,cc),'ok','MarkerSize',5,'MarkerFaceColor', 'y');
            cc = cc +1;
        end
    end
    pause(.1);
end


%% EKF with estimated Bernouilli
bernoulli = 0;
if bernoulli
    
    cvXhat = [];

    syms  z1 z2 z3 z4;
    V = [z1; z2; z3; z4];

    % Initialization:
    cvX0 = zeros(4,target_sick);
    mP0 = zeros(4,4,target_sick);
    for i =1:target_sick
        mP0(:,:,i) = 100*eye(4);
    end
    % Bernouilli vector
    B = ones(nCams,1); %all cameras can see well
    beta = ones(nCams,1);
    beta_MP0 = 1*eye(nCams);
    
    F_beta = eye(nCams);

    x_vect = zeros(4,N,target_sick);

    for k = 1:N %for each step 

        % Bernoulli estimation considering all info given by the track of more
        % targets
        [beta, beta_MP0] = Bernoulli_estimation(beta, beta_MP0, cvX0, xyCams, dist_extended(k,:,:), camera_activated_extended(k,:,:)); 

        for i = 1 :target_sick      % only for sick target compute the estimation
            if dk(sick_pos(i)) 
                ED = {};            % Euclidean Distance
                indexes = [];       % misure SOLO delle telecamere che vedono il target
                c = 1;

                for j = 1:nCams
                    % Prepare the function to calculate the jacobian
                    if camera_activated_extended(k,j,sick_pos(i))       % check if camera can see                                
                        ED{c} = @(z1, z2, z3, z4) sqrt((XYCAMS(j,1) - z1)^2 + (XYCAMS(j,2) - z2)^2 );
                        c = c + 1;
                    end
                end
                % All measurements of cameras that can see the target
                Yk = dist_extended(k,(find(camera_activated_extended(k,:,sick_pos(i)))), sick_pos(i));

                Rrep = repmat(r, 1, length(Yk));
                Rcell = mat2cell(Rrep, size(r,1), repmat(size(r, 2), 1, length(Yk)));
                R = blkdiag(Rcell{:}); % Covariance matrix for all sensors

                % Kalman Filter step
                [cvXhat, mP] = EKF(F, G, ED, V, Yk, cvX0(:,i), mP0(:,:,i), R, Q, beta);

                % save the position
                x_vect(:,k,i) = cvXhat; 
                % uptade the intial conditions

                cvX0(:,i) = cvXhat;
                mP0(:,:,i) = mP;
            end
        end
    end


    % Grafico i risultati:
    figure(5);
    hold on;
    grid on;
    axis([-10 Mx+10 -10 My+10])
    hold on;
    plot([0;Mx], [0, 0],'k','LineWidth',2); 
    plot([0;Mx], [My, My],'k','LineWidth',2); 
    plot([0;0], [0, My],'k','LineWidth',2); 
    plot([Mx;Mx], [0, My],'k','LineWidth',2);
    hold on;
    rectangle('Position', [pos_obs_x,pos_obs_y,size_obs_x,size_obs_y], 'FaceColor', 'k')
    hold on;
    for k =1 : N   
        cc = 1;
        for i = 1:nTarget
            plot(X(k,1,i),X(k,2,i),'ok','MarkerSize',5,'MarkerFaceColor',v(i,:));
            if dk(i)
                plot(x_vect(1, k,cc), x_vect(2, k,cc),'ok','MarkerSize',5,'MarkerFaceColor', 'y');
                cc = cc +1;
            end
        end
        pause(.1);
    end
end