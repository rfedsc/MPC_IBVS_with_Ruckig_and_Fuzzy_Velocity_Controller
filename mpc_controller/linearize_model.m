function [A,B] = linearize_model(s,tau,Z,Ts)
    %Initialization
    n = length(s);%Dimension of state
    m = length(tau);%Dimension of control input
    %Calculate interaction matrix
    Ls = Calculate_Ls(s,Z);
    %Calculate Ak (state Jacobian matrix)
    delta = 1e-4;%Small perturbation
    A = eye(n);
    for i = 1:n
        s_perturb = s;
        s_perturb(i) = s_perturb(i) + delta;
        %Calculate perturbed Ls
        Ls_perturb = Calculate_Ls(s_perturb,Z);
        %Numerical differentiation to calculate state derivative
        dLs = (Ls_perturb-Ls)/delta;
        %Update column of Ak
        A(:,i) = Ts*dLs*tau;
    end
    %Calculate Bk (control input Jacobian matrix)
    B = Ts*Ls;%Bk is directly derived from the interaction matrix
end