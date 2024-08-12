function [vertices] = IntegratorBounds(p, dt, bounds, const, v_prev)
    
    if nargin < 4
        plb_alt = bounds.plb;
        pub_alt = bounds.pub;
        vlb_alt = bounds.vlb;
        vub_alt = bounds.vub;
    else
        % plb_alt = bounds.plb;
        % pub_alt = bounds.pub;
        % vlb_alt = max([v_prev + const(2)*(bounds.vlb-v_prev), const(1)*(bounds.plb - p)], [], 2);
        % vub_alt = min([v_prev + const(2)*(bounds.vub-v_prev), const(1)*(bounds.pub - p)], [], 2);
        % plb_alt = p + const(1)*(bounds.plb-p);
        % pub_alt = p + const(1)*(bounds.pub-p);
        % vlb_alt = bounds.vlb;
        % vub_alt = bounds.vub;
        plb_alt = bounds.plb;
        pub_alt = bounds.pub;
        vlb_alt = v_prev + const(2)*(max([bounds.vlb, const(1)*(bounds.plb - p)], [], 2)-v_prev);
        vub_alt = v_prev + const(2)*(min([bounds.vub, const(1)*(bounds.pub - p)], [], 2)-v_prev);
    end

    % vertices arranged in columns

    % pn = p + dt*v
    threshold = 0e-3;

    pub_cand = [min([bounds.pub,pub_alt],[],2)-threshold; p+dt*(min([bounds.vub,vub_alt],[],2)-threshold)];
    plb_cand = [max([bounds.plb,plb_alt],[],2)+threshold; p+dt*(max([bounds.vlb,vlb_alt],[],2)+threshold)];
    
    if length(p) == 1
        vertices = [min(pub_cand), max(plb_cand)];
        if vertices(1) < vertices(2)
            wtf
        end
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