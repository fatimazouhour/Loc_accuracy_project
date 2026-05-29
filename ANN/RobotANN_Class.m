classdef RobotANN < handle
    properties
        net
        is_trained = false
        nInputs
        nOutputs = 2
    end
    
    methods
        function obj = RobotANN(nInputs)
            if nargin < 1
                nInputs = 10; % Eq. (3) input vector
            end

            obj.nInputs = nInputs;

            obj.net = feedforwardnet([10 5]);

            obj.net.layers{1}.transferFcn = 'logsig';
            obj.net.layers{2}.transferFcn = 'logsig';
            obj.net.layers{3}.transferFcn = 'purelin';

            % Quasi-Newton / secant-like training
            obj.net.trainFcn = 'trainoss';

            % Regularization
            obj.net.performParam.regularization = 0.09;

            % RMSE = 1e-4 -> MSE = 1e-8
            obj.net.trainParam.goal = 1e-8;

            % Optional training settings
            obj.net.trainParam.epochs = 1000;
            obj.net.trainParam.showWindow = true;
        end

        function trainANN(obj, inputs, targets)

            if size(inputs,2) ~= obj.nInputs
                error('Input data must have %d columns.', obj.nInputs);
            end

            if size(targets,2) ~= obj.nOutputs
                error('Targets must have 2 columns: x and y.');
            end

            [obj.net, tr] = train(obj.net, inputs', targets');

            obj.is_trained = true;

            if tr.bestPerf > obj.net.trainParam.goal
                warning('Training finished, but RMSE goal was not fully reached.');
            end
        end

        function pos_pred = predict(obj, sensor_data)

            if ~obj.is_trained
                error('ANN is not trained yet.');
            end

            if size(sensor_data,2) ~= obj.nInputs
                error('Sensor data must have %d columns.', obj.nInputs);
            end

            pos_pred = obj.net(sensor_data')';
        end
    end
end
