function [vertices] = DoubleIntegratorBounds(p, v, dt, bounds, const, a_prev)
    
    if nargin < 5
        alb_alt = bounds.alb;
        aub_alt = bounds.aub;
        vlb_alt = bounds.vlb;
        vub_alt = bounds.vub;
    elseif nargin < 6
        % arg1 is 'const', low pass filter on velocity
        vlb_alt = v + const(2)*(bounds.vlb-v);
        vub_alt = v + const(2)*(bounds.vub-v);
        alb_alt = bounds.alb;
        aub_alt = bounds.aub;
    else
        % const is between 0 and 1
        % a <= a_prev + const*(bounds.aub-a_prev)
        % a >= a_prev + const*(bounds.alb-a_prev)

        % vlb_alt = bounds.vlb;
        % vub_alt = bounds.vub;
        % alb_alt = a_prev + const(3)*(bounds.alb-a_prev);
        % aub_alt = a_prev + const(3)*(bounds.aub-a_prev);
        
        vlb_alt = v + const(2)*(bounds.vlb-v);
        vub_alt = v + const(2)*(bounds.vub-v);
        alb_alt = a_prev + const(3)*(bounds.alb-a_prev);
        aub_alt = a_prev + const(3)*(bounds.aub-a_prev);
    end

    % vertices arranged in columns

    % pn = p + dt*v + 0.5*dt^2*a
    % vn = v + dt*a 
    % pn = p + 0.5*dt*v + 0.5*dt*vn
    threshold = 0e-3;

    pub_cand = [bounds.pub-threshold; p+0.5*dt*v+0.5*dt*(min([bounds.vub,vub_alt],[],2)-threshold); p+dt*v+0.5*dt^2*(min([bounds.aub,aub_alt],[],2)-threshold)];
    plb_cand = [bounds.plb+threshold; p+0.5*dt*v+0.5*dt*(max([bounds.vlb,vlb_alt],[],2)+threshold); p+dt*v+0.5*dt^2*(max([bounds.alb,alb_alt],[],2)+threshold)];
    % pub_cand = [bounds.pub-threshold; p+0.5*dt*v+0.5*dt*(min([bounds.vub,vub_alt],[],2)-threshold)];
    % plb_cand = [bounds.plb+threshold; p+0.5*dt*v+0.5*dt*(max([bounds.vlb,vlb_alt],[],2)+threshold)];
    
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