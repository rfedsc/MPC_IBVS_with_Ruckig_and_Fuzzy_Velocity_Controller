function [G_fov,h_fov] = get_fov_constrains(s0,z_start,Np,K,F,image_width,image_height,A,B)
    % Camera intrinsic parameters
    fx = K(1,1);
    fy = K(2,2);
    cx = K(1,3);
    cy = K(2,3);

    % Normalized image plane boundaries (u/v in normalized coordinates)
    u_min = -cx/fx;
    u_max = (image_width-cx)/fx;
    v_min = -cy/fy;
    v_max = (image_height-cy)/fy;

    % Number of feature points (each feature has u/v coordinates)
    n_feat = length(s0)/2;

    % Construct prediction matrices A_big and B_big for Np steps
    n_s = length(s0);       
    n_tau = size(B,2);      
    A_big = zeros(n_s*Np,n_s);
    B_big = zeros(n_s*Np,n_tau*Np);

    for i = 1:Np
        A_power = A^i;
        A_big((i-1)*n_s+1:i*n_s,:) = A_power;
        for j = 1:i
            AB = A^(i-j)*B;
            B_big((i-1)*n_s+1:i*n_s,(j-1)*n_tau+1:j*n_tau) = AB;
        end
    end

    % Selection matrices: extract u and v components from feature vector
    Cu = zeros(n_feat,n_s);
    Cv = zeros(n_feat,n_s);
    for i = 1:n_feat
        Cu(i,(i-1)*2+1) = 1; 
        Cv(i,(i-1)*2+2) = 1; 
    end
    % Construct block diagonal Cu_big/Cv_big (cover entire Np prediction horizon)
    Cu_big = kron(eye(Np),Cu);
    Cv_big = kron(eye(Np),Cv);
    
    % Construct linear FOV constraint matrices G and h
    G_fov = [ Cu_big*B_big;    
             -Cu_big*B_big;    
              Cv_big*B_big;    
              -Cv_big*B_big];  

    % Expand u/v boundary limits to Np prediction steps
    u_max_vec = repmat(u_max,n_feat*Np,1);
    u_min_vec = repmat(u_min,n_feat*Np,1);
    v_max_vec = repmat(v_max,n_feat*Np,1);
    v_min_vec = repmat(v_min,n_feat*Np,1);

    % Calculate offset terms from initial feature vector s0
    offset_u = Cu_big*A_big*s0;
    offset_v = Cv_big*A_big*s0;

    % Construct h_fov vector (bound limits minus offset terms)
    h_fov = [u_max_vec-offset_u;
            -u_min_vec+offset_u;
            v_max_vec-offset_v;
            -v_min_vec+offset_v];
end