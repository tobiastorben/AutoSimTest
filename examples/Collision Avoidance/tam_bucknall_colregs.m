%© Norwegian University of Science and Technology (NTNU),
%Department of Marine Technology.
%The software is developed under the ORCAS Project.
%Author: Tobias Rye Torben.

function encounter_type = tam_bucknall_colregs(eta_os, eta_ts)
    r = eta_ts(1:2) - eta_os(1:2);
    rel_bearing = rad2deg(wrapTo2Pi(atan2(r(2), r(1))-eta_os(3)));
    rel_heading = rad2deg(wrapTo2Pi(eta_ts(3)-eta_os(3)));
    
    %Determine heading region
    if rel_heading >= 67.5 && rel_heading < 90
        heading_region = 2;
    elseif rel_heading >= 90 && rel_heading < 157.5
        heading_region = 3;
    elseif rel_heading >= 157.5 && rel_heading < 202.5
        heading_region = 4;
    elseif rel_heading >= 202.5 && rel_heading < 270
        heading_region = 5;
    elseif rel_heading >= 270 && rel_heading < 292.5
        heading_region = 6;
    else
        heading_region = 1;
    end
    
    %Determine bearing region
    if rel_bearing >= 22.5 && rel_bearing < 90
        bearing_region = 2;
    elseif rel_bearing >= 90 && rel_bearing < 112.5
        bearing_region = 3;
    elseif rel_bearing >= 112.5 && rel_bearing < 247.5
        bearing_region = 4;
    elseif rel_bearing >= 247.5 && rel_bearing < 270
        bearing_region = 5;
    elseif rel_bearing >= 270 && rel_bearing < 337.5
        bearing_region = 6;
    else
        bearing_region = 1;
    end
    
    lookup_table = ... 
    ["OVERTAKING" "STAND_ON" "STAND_ON" "HEADON" "GIVE_WAY" "GIVE_WAY";
     "OVERTAKING" "NO_CONFLICT" "NO_CONFLICT" "HEADON" "GIVE_WAY" "GIVE_WAY";
     "OVERTAKEN" "NO_CONFLICT" "NO_CONFLICT" "NO_CONFLICT" "GIVE_WAY" "GIVE_WAY";
     "OVERTAKEN" "STAND_ON" "NO_CONFLICT" "NO_CONFLICT" "NO_CONFLICT" "GIVE_WAY";
     "OVERTAKEN" "STAND_ON" "STAND_ON" "NO_CONFLICT" "NO_CONFLICT" "NO_CONFLICT";
     "OVERTAKING" "STAND_ON" "STAND_ON" "HEADON" "NO_CONFLICT" "NO_CONFLICT";
    ];
    
    encounter_type = lookup_table(bearing_region,heading_region);
end