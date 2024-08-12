function [pn] = IntegratorForwardSolution(v, p, dt, bounds)
    
    % pn = p + dt*v

    pn = p + dt*v;

    threshold = 1e-3;
    

    if length(p) == 1

        if pn < bounds.plb - threshold || pn > bounds.pub + threshold
            warning('Position bounds (reconfiguration system) violated.');
        end
    
        if v < bounds.vlb - threshold || v > bounds.vub + threshold
            warning('Velocity bounds (reconfiguration system) violated.');
        end
        
    elseif length(p) == 2
    
        if sum(pn < bounds.plb - threshold) || sum(pn > bounds.pub + threshold)
            warning('Position bounds (reconfiguration system) violated.');
        end
    
        if sum(v < bounds.vlb - threshold) || sum(v > bounds.vub + threshold)
            warning('Velocity bounds (reconfiguration system) violated.');
        end

    else
        error('Only 1-D or 2-D cases are supported.')
    end

end