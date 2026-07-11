classdef RLSEstimator < matlab.System
    %RLSESTIMATOR Recursive Least Squares parameter estimator.
    %   Standard RLS with (optional) exponential forgetting, used as the
    %   shared online parameter estimator for ddc.str.IndirectSTRController
    %   and available standalone for any linear-in-parameters model
    %   y_k = phi_k' * theta + e_k.
    %
    %   theta_k = theta_{k-1} + K_k * (y_k - phi_k' * theta_{k-1})
    %   K_k     = P_{k-1} * phi_k / (lambda + phi_k' * P_{k-1} * phi_k)
    %   P_k     = (P_{k-1} - K_k * phi_k' * P_{k-1}) / lambda
    %
    %   Example (MATLAB):
    %       est = ddc.common.RLSEstimator('NumParameters', 3, ...
    %                                      'ForgettingFactor', 0.98);
    %       [theta, err] = est.step(phi, y);
    %
    %   See also ddc.str.IndirectSTRController, ddc.mfac.MFACController.

    properties (Nontunable)
        NumParameters    (1,1) double {mustBePositive, mustBeInteger} = 1
    end

    properties
        ForgettingFactor (1,1) double {mustBeGreaterThan(ForgettingFactor,0), ...
                                        mustBeLessThanOrEqual(ForgettingFactor,1)} = 1
        InitialCovarianceGain (1,1) double {mustBePositive} = 1e4
    end

    properties (Access = private)
        Theta   % current parameter estimate, NumParameters-by-1
        P       % covariance matrix, NumParameters-by-NumParameters
    end

    methods
        function obj = RLSEstimator(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            n = obj.NumParameters;
            obj.Theta = zeros(n, 1);
            obj.P = obj.InitialCovarianceGain * eye(n);
        end

        function resetImpl(obj)
            n = obj.NumParameters;
            obj.Theta = zeros(n, 1);
            obj.P = obj.InitialCovarianceGain * eye(n);
        end

        function [theta, predictionError] = stepImpl(obj, phi, y)
            phi = phi(:);
            lambda = obj.ForgettingFactor;

            Pphi = obj.P * phi;
            denom = lambda + phi.' * Pphi;
            K = Pphi / denom;

            predictionError = y - phi.' * obj.Theta;
            obj.Theta = obj.Theta + K * predictionError;
            obj.P = (obj.P - K * phi.' * obj.P) / lambda;

            theta = obj.Theta;
        end

        function [sz1, sz2] = getOutputSizeImpl(obj)
            sz1 = [obj.NumParameters, 1];
            sz2 = [1, 1];
        end

        function [dt1, dt2] = getOutputDataTypeImpl(~)
            dt1 = 'double';
            dt2 = 'double';
        end

        function [cp1, cp2] = isOutputComplexImpl(~)
            cp1 = false;
            cp2 = false;
        end

        function [fz1, fz2] = isOutputFixedSizeImpl(~)
            fz1 = true;
            fz2 = true;
        end

        function num = getNumInputsImpl(~)
            num = 2;
        end

        function num = getNumOutputsImpl(~)
            num = 2;
        end
    end
end
