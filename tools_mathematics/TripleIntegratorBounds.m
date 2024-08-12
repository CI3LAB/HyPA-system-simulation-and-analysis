function [vertices] = TripleIntegratorBounds(p, v, a, dt, bounds)

    % vertices arranged in columns

    % pn = p + (2/3)*dt*v + (1/6)*dt^2*a + (1/3)*dt*vn
    % pn = p + dt*v + (1/3)*dt^2*a + (1/6)*dt^2*an
    % pn = p + dt*v + (1/2)*dt^2*a + (1/6)*dt^3*j

    threshold = 1e-3;

    pub_cand = [bounds.pub-threshold; p+(2/3)*dt*v+(1/6)*dt^2*a+(1/3)*dt*(bounds.vub-threshold); p+dt*v+(1/3)*dt^2*a+(1/6)*dt^2*(bounds.aub-threshold); p+dt*v+(1/2)*dt^2*a+(1/6)*dt^3*(bounds.jub-threshold)];
    plb_cand = [bounds.plb+threshold; p+(2/3)*dt*v+(1/6)*dt^2*a+(1/3)*dt*(bounds.vlb+threshold); p+dt*v+(1/3)*dt^2*a+(1/6)*dt^2*(bounds.alb+threshold); p+dt*v+(1/2)*dt^2*a+(1/6)*dt^3*(bounds.jlb+threshold)];
    
    if length(p) == 1
        vertices = [min(pub_cand), max(plb_cand)];
    elseif length(p) == 2
        plb = max(plb_cand, [], 2);
        pub = min(pub_cand, [], 2);
        vertices = [plb(1), plb(2);
                    plb(1), pub(2);
                    pub(1), plb(2);
                    pub(1), pub(2)]';
    else
        error('Only 1-D or 2-D cases are supported.');
    end
    
end