function L_library = build_L_library(rho_samples)
    n_samples = size(rho_samples,1);
    L_library = zeros(6,6,n_samples);
    for i = 1:n_samples
        rho_i = rho_samples(i,:)';
        s_i = rho_i(1:end-1);
        Z_i = rho_i(end);
        L_library(:,:,i) = Calculate_Ls(s_i,Z_i);
end
