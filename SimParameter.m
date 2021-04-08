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