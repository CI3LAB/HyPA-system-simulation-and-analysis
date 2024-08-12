function [vn, an, j] = TripleIntegratorInverseSolution(pn, p, v, a, dt, bounds)
    
    % pn = p + (2/3)*dt*v + (1/6)*dt^2*a + (1/3)*dt*vn
    % pn = p + dt*v + (1/3)*dt^2*a + (1/6)*dt^2*an
    % pn = p + dt*v + (1/2)*dt^2*a + (1/6)*dt^3*j


    vn = (pn-p-(2/3)*dt*v-(1/6)*dt^2*a)/((1/3)*dt);
    an = (pn-p-dt*v-(1/3)*dt^2*a)/((1/6)*dt^2);
    j = (pn-p-dt*v-(1/2)*dt^2*a)/((1/6)*dt^3);

    threshold = 1e-3;


    if length(p) == 1
    
        if pn < bounds.plb - threshold || pn > bounds.pub + threshold
            warning('Position bounds (reconfiguration system) violated.');
        end
    
        if vn < bounds.vlb - threshold || vn > bounds.vub + threshold
            warning('Velocity bounds (reconfiguration system) violated.');
        end
    
        if an < bounds.alb - threshold || an > bounds.aub + threshold
            warning('Acceleration bounds (reconfiguration system) violated.');
        end
    
        if j < bounds.jlb - threshold || j > bounds.jub + threshold
            warning('Jerk bounds (reconfiguration system) violated.');
        end
        
    elseif length(p) == 2
    
        if sum(pn < bounds.plb - threshold) || sum(pn > bounds.pub + threshold)
            warning('Position bounds (reconfiguration system) violated.');
        end
    
        if sum(vn < bounds.vlb - threshold) || sum(vn > bounds.vub + threshold)
            warning('Velocity bounds (reconfiguration system) violated.');
        end
    
        if sum(an < bounds.alb - threshold) || sum(an > bounds.aub + threshold)
            warning('Acceleration bounds (reconfiguration system) violated.');
        end
    
        if sum(j < bounds.jlb - threshold) || sum(j > bounds.jub + threshold)
            warning('Jerk bounds (reconfiguration system) violated.');
        end

    else
        error('Only 1-D or 2-D cases are supported.')
    end

end