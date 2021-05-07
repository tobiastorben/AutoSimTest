%© Norwegian University of Science and Technology (NTNU),
%Department of Marine Technology.
%The software is developed under the ORCAS Project.
%Author: Tobias Rye Torben

classdef SimParameter
   properties
      Name
      Values
   end
   methods
     function obj = SimParameter(name, values)
         obj.Name = name;
         obj.Values = values;
     end
   end
end