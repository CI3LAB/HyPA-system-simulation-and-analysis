classdef PID < ControllerSISO
    properties
        ep_max % ==0 for cable
        ep_min
        ev_max % ==0 for cable
        ev_min
        kp
        kd
        isCable
        pp
        pv
    end

    methods
        % A constructor for the controller base.
        function pid = PID(u_max, ep_max, ev_max, kp, kd)
            pid@ControllerSISO(u_max);
            pid.ep_min = -ep_max;
            pid.ep_max = ep_max;
            pid.ev_min = -ev_max;
            pid.ev_max = ev_max;
            pid.kp = kp;
            pid.kd = kd;
            pid.isCable = false;
            pid.pp = 1;
            pid.pv = 1;
        end
        
        function setGainCurve(obj, pp, pv)
            obj.pp = pp;
            obj.pv = pv;
        end
        
        function setUmin(obj, umin)
            obj.u_min = umin;
        end
        
        function setCableLengthController(obj)
            obj.isCable = true;
            if obj.u_min < 0
                obj.u_min = 0;
            end
        end

        function u  = executeFunction(obj, p, v, pd, vd)
            if obj.isCable
                ep = p - pd;
                ev = v - vd;
                if ep < 0.0
                    ep = 0.0;
                    ev = 0.0;
%                 elseif ev < 0.0
%                     ev = 0.0;
                end
            else
                ep = pd - p;
                ev = vd - v;
            end
            [ep, ep_ratio, ep_lim] = obj.saturate(ep, obj.ep_min, obj.ep_max);
            [ev, ev_ratio, ev_lim] = obj.saturate(ev, obj.ev_min, obj.ev_max);
            ep_ = sign(ep)*ep_lim*ep_ratio^obj.pp;
            ev_ = sign(ev)*ev_lim*ev_ratio^obj.pv;
            u = obj.kp * ep_ + obj.kd * ev_;
        end
    end

end
