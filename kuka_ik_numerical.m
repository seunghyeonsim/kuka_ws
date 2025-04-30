function [q,lambdalist]=kuka_ik_numerical(Tinit, q,Tdes)
eps_ep = 5e-4;
eps_eo= 5e-4;
p_list = {[0; 0; 0], [0.330; 0; 0.645], [1.480; 0; 0.645], [1.480; 0; 0.76], [2.7; 0; 0.76], [2.94; 0; 0.76]};
z_list = {[0; 0; 1], [0; 1; 0], [0; 1; 0], [1; 0; 0], [0; 1; 0], [1; 0; 0]};
for i=1:6
    lambda = [cross(p_list{i},z_list{i}); z_list{i}];
    lambdalist(:,i) = lambda;
end

max_iter = 1000;
k = 0.83;
niter = 1;
while true
    [T,Jb,~,~,~] = axisKinematics(Tinit, lambdalist, q, zeros(size(q)), zeros(size(q)));
    Tbd = TransInv(T)*Tdes;
    lambda = logSE3(Tbd);
    epsilon = 0.01;

    % Jb_inv = pinv(Jb);
    % q = q + k*Jb_inv*lambda;
    Jbinv_DLS = Jb'/(Jb*Jb' + epsilon*eye(6));
    q = q + k*Jbinv_DLS*lambda;
    ep = lambda(1:3);
    eo = lambda(4:6);
    if norm(ep) < eps_ep && norm(eo) < eps_eo
        break
    end
    niter = niter + 1;
    if niter >= max_iter
        break
    end
end