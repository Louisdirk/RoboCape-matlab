classdef KeyboardController < Controller
    
    properties
        
    end
    
    methods
        
        
        function obj = KeyboardController()
            
            obj = obj@Controller();
            
            set(gcf, 'KeyPressFcn', @keyPressCallback);
            
            global pressed_key
            pressed_key = 'x';
        end
        
        
        function u = computeInput(obj,t,x)
            
            global pressed_key
%             disp('pressed')
%             disp(pressed_key);
            
            u(1,1) = 1;
            
            switch pressed_key
                case 'w'
                    u(2,1) = 0;
                case 'a'
                    u(2,1) = 0.25;
                case 'd'
                    u(2,1) = -0.25;
                case 's'
                    u(2,1) = 0;
                    u(1,1) = -u(1);
                otherwise
                    u(1,1) = 0;
                    u(2,1) = 0;
            end
        end
        
        
    end
    
end

