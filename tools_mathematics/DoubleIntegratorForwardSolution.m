function [pn, vn] = DoubleIntegratorForwardSolution(a, p, v, dt, bounds)
    
    % pn = p + dt*v + 0.5*dt^2*a
    % vn = v + dt*a 
    % pn = p + 0.5*dt*v + 0.5*dt*vn

    pn = p + dt*v + 0.5*dt^2*a;
    vn = v + dt*a;

    threshold = 1e-3;
    

    if length(p) == 1

        if pn < bounds.plb - threshold || pn > bounds.pub + threshold
            warning('Position bounds (reconfiguration system) violated.');
        end
    
        if vn < bounds.vlb - threshold || vn > bounds.vub + threshold
            warning('Velocity bounds (reconfiguration system) violated.');
        end
    
        if a < bounds.alb - threshold || a > bounds.aub + threshold
            warning('Acceleration bounds (reconfiguration system) violated.');
        end
        
    elseif length(p) == 2
    
        if sum(pn < bounds.plb - threshold) || sum(pn > bounds.pub + threshold)
            warning('Position bounds (reconfiguration system) violated.');
        end
    
        if sum(vn < bounds.vlb - threshold) || sum(vn > bounds.vub + threshold)
            warning('Velocity bounds (reconfiguration system) violated.');
        end
    
        if sum(a < bounds.alb - threshold) || sum(a > bounds.aub + threshold)
            warning('Acceleration bounds (reconfiguration system) violated.');
        end

    else
        error('Only 1-D or 2-D cases are supported.')
    end

end