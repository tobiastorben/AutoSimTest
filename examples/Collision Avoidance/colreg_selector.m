%© Norwegian University of Science and Technology (NTNU),
%Department of Marine Technology.
%The software is developed under the ORCAS Project.
%Author: Tobias Rye Torben.
function [headon, overtaking, overtaken, standon, giveway] = colreg_selector(cpa, tcpa, eta_os, eta_ts)
    cpa_conf = 100.0;
    tcpa_conf = 100.0;
    tcpa_neg = 1e10;
    
    state = 'NO_CONFLICT';
    headon = -ones(size(cpa));
    overtaking = -ones(size(cpa));
    overtaken = -ones(size(cpa));
    standon = -ones(size(cpa));
    giveway = -ones(size(cpa));
    
    for i = 1:length(cpa)
        switch state
            case 'NO_CONFLICT'
                if cpa(i) <= cpa_conf && tcpa(i) <= tcpa_conf
                    state = 'INBOUND';
                else
                    state = 'NO_CONFLICT';
                end
            case 'INBOUND'
                state = tam_bucknall_colregs(eta_os(:,i), eta_ts(:,i));
            case 'HEADON'
                headon(i) = 1;
                if tcpa(i) >= tcpa_neg
                    state = 'NO_CONFLICT';
                else
                    state = 'HEADON';
                end
            case 'OVERTAKING'
                overtaking(i) = 1;
                if tcpa(i) >= tcpa_neg
                    state = 'NO_CONFLICT';
                else
                    state = 'OVERTAKING';
                end
            case 'OVERTAKEN'
                overtaken(i) = 1;
                if tcpa(i) >= tcpa_neg
                    state = 'NO_CONFLICT';
                else
                    state = 'OVERTAKEN';
                end
            case 'STAND_ON'
                standon(i) = 1;
                if tcpa(i) >= tcpa_neg
                    state = 'NO_CONFLICT';
                else
                    state = 'STAND_ON';
                end
            case 'GIVE_WAY'
                giveway(i) = 1;
                if tcpa(i) >= tcpa_neg
                    state = 'NO_CONFLICT';
                else
                    state = 'GIVE_WAY';
                end
            otherwise
                error('UNDEFINED STATE!')
        end
    end
end