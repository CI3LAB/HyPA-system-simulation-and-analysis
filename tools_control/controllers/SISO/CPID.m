classdef CPID < ControllerSISO
    properties
        vd_max % ==0 for cable
        vd_min
        ep_max % ==0 for cable
        ep_min
        ev_max % ==0 for cable
        ev_min
        kp
        kv
        isCable
        pp
        pv
    end

    methods
        % A constructor for the controller base.
        function cpid = CPID(u_max, vd_max, ep_max, ev_max, kp, kv)
            cpid@ControllerSISO(u_max);
            cpid.vd_min = -vd_max;
            cpid.vd_max = vd_max;
            cpid.ep_min = -ep_max;
            cpid.ep_max = ep_max;
            cpid.ev_min = -ev_max;
            cpid.ev_max = ev_max;
            cpid.kp = kp;
            cpid.kv = kv;
            cpid.isCable = false;
            cpid.pp = 1;
            cpid.pv = 1;
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
        end

        function u  = executeFunction(obj, p, v, pd, vd)
            if obj.isCable 
                ep = p - pd;
                if ep < 0.0
                    u = 0;
                    return
                end
                [ep, ep_ratio, ep_lim] = obj.saturate(ep, obj.ep_min, obj.ep_max);
                vcmd = obj.kp*(sign(ep)*ep_lim*ep_ratio^obj.pp);
                vd = vd - vcmd;
                vd = obj.saturate(vd, obj.vd_min, obj.vd_max);
                ev = v - vd;
                [ev, ev_ratio, ev_lim] = obj.saturate(ev, obj.ev_min, obj.ev_max);
                u = obj.kv*(sign(ev)*ev_lim*ev_ratio^obj.pv);
            else
                ep = pd - p;
                [ep, ep_ratio, ep_lim] = obj.saturate(ep, obj.ep_min, obj.ep_max);
                vcmd = obj.kp*(sign(ep)*ep_lim*ep_ratio^obj.pp);
                vd = vd + vcmd;
                vd = obj.saturate(vd, obj.vd_min, obj.vd_max);
                ev = vd - v;
                [ev, ev_ratio, ev_lim] = obj.saturate(ev, obj.ev_min, obj.ev_max);
                u = obj.kv*(sign(ev)*ev_lim*ev_ratio^obj.pv);
            end
        end
    end

end
