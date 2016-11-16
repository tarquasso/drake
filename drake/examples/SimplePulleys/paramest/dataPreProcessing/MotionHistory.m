classdef MotionHistory < handle
    %MOTIONHISTORY Class containing the history of the arm's shape
    %   This is a handle class, i.e. objects are always passed by reference
    %   rather than value. As a result the values can be efficiently
    %   updated on the go without copying the whole history.
    
    properties
        S   % Number of bodies in the arm
        N   % Size of samples buffer
        i   % Current index
        
        timestamps      % Timestamps of the state
        frameID
        frameTime
        objectPosition
        plannerResults
    end
    
    methods
        function obj = MotionHistory(S, N)
            % Create the initial object
            obj.S = S;
            obj.N = N;
            obj.i = 1;
            
            obj.timestamps = NaN(N,1);
            obj.frameID = NaN(N,1);
            obj.frameTime = NaN(N,1);
            obj.objectPosition = NaN(N,3);
            
        end
        
        function add(obj, timestamp,frameID,frameTime, objectPosition)
            % Add a new measurement
            obj.timestamps(obj.i, 1) = timestamp;
            obj.frameID(obj.i, 1) = frameID;
            obj.frameTime(obj.i, 1) = frameTime;
            obj.objectPosition(obj.i, :) = objectPosition;
            
            % Update the count
            obj.i = mod(obj.i + 1, obj.N+1);
            if obj.i == 0
                obj.i = obj.N;
            end
        end
                
        %Destructor
        function delete(obj)
        end
        
    end
    
end
