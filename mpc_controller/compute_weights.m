function w = compute_weights(rho,rho_samples,sigma)
    n = size(rho_samples,1);
    w = zeros(n,1);
    for i = 1:n
        d = norm(rho-rho_samples(i,:)');
        w(i) = exp(-d^2/(2*sigma^2));
    end
    w = w/sum(w);
end
