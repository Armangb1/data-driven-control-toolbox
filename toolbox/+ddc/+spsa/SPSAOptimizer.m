classdef SPSAOptimizer < matlab.System
    %SPSAOPTIMIZER Simultaneous Perturbation Stochastic Approximation.
    %   Online, gradient-free stochastic optimizer (Spall, "Multivariate
    %   Stochastic Approximation Using a Simultaneous Perturbation
    %   Gradient Approximation", 1992) for tuning a parameter vector from
    %   noisy scalar loss measurements only (no gradient required) --
    %   applicable to tuning gains of any controller in this toolbox
    %   (STR, MFAC, UFC candidate gains, etc.).
    %
    %   Each gradient estimate requires two loss evaluations, so the
    %   block operates as a cycle driven by successive calls to step().
    %   Two cycle modes are available (set via ParameterUpdateMode):
    %
    %   "immediate" (default) -- 3-phase cycle:
    %     Phase 1: emit theta + c_k*Delta_k  ("ParamToApply")
    %     Phase 2: receive loss at theta+c_k*Delta_k, emit theta - c_k*Delta_k
    %     Phase 3: receive loss at theta-c_k*Delta_k, form the SPSA
    %              gradient estimate, update theta, emit the new theta
    %              estimate (back to Phase 1)
    %
    %   "deferred" -- 2-phase cycle:
    %     Phase 1: (if a previous J+ is stored: receive J- at theta-c_k*Delta_k,
    %              form the gradient, update theta), then compute new
    %              c_k,Delta_k, emit theta + c_k*Delta_k
    %     Phase 2: receive J+ at theta+c_k*Delta_k, store it, emit
    %              theta - c_k*Delta_k (back to Phase 1)
    %
    %   Standard decaying gain sequences are used:
    %       a_k = a / (k+1+Abar)^alpha
    %       c_k = c / (k+1)^gamma
    %
    %   Usable as a plain MATLAB object or as a Simulink "MATLAB System"
    %   block. Wire: ParamToApply -> (system under tuning) -> loss metric
    %   -> LossMeasurement input.
    %
    %   Example (MATLAB):
    %       opt = ddc.spsa.SPSAOptimizer('NumParameters', 2, ...
    %           'InitialTheta', [1;1]);
    %       for k = 1:300
    %           [paramToApply, thetaHat] = opt.step(loss); % apply paramToApply, measure loss
    %       end

    properties (Nontunable)
        NumParameters (1,1) double {mustBePositive, mustBeInteger} = 1
        InitialTheta (:,1) double = 0
        ParameterUpdateMode (1,1) string {mustBeMember(ParameterUpdateMode, ["immediate","deferred"])} = "immediate"
    end

    properties
        ATuning (1,1) double {mustBePositive} = 0.1    % numerator of a_k
        CTuning (1,1) double {mustBePositive} = 0.1    % numerator of c_k
        ACommon (1,1) double {mustBeNonnegative} = 10  % stability constant Abar
        Alpha   (1,1) double {mustBePositive} = 0.602  % standard SPSA exponent
        Gamma   (1,1) double {mustBePositive} = 0.101  % standard SPSA exponent
    end

    properties (Access = private)
        Theta_
        K_
        Phase_
        Delta_
        Ck_
        LossPlus_
        HasStoredLoss_
    end

    methods
        function obj = SPSAOptimizer(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            n = obj.NumParameters;
            if isscalar(obj.InitialTheta)
                obj.Theta_ = repmat(obj.InitialTheta, n, 1);
            else
                obj.Theta_ = obj.InitialTheta(:);
            end
            obj.K_ = 0;
            obj.Phase_ = 1;
            obj.Delta_ = ones(n, 1);
            obj.Ck_ = obj.CTuning;
            obj.LossPlus_ = 0;
            obj.HasStoredLoss_ = false;
        end

        function resetImpl(obj)
            n = obj.NumParameters;
            if isscalar(obj.InitialTheta)
                obj.Theta_ = repmat(obj.InitialTheta, n, 1);
            else
                obj.Theta_ = obj.InitialTheta(:);
            end
            obj.K_ = 0;
            obj.Phase_ = 1;
            obj.Delta_ = ones(n, 1);
            obj.Ck_ = obj.CTuning;
            obj.LossPlus_ = 0;
            obj.HasStoredLoss_ = false;
        end

        function [paramToApply, thetaEstimate] = stepImpl(obj, lossMeasurement)
            n = obj.NumParameters;

            if obj.ParameterUpdateMode == "deferred"
                % ---- 2-phase deferred cycle ----
                switch obj.Phase_
                    case 1
                        if obj.HasStoredLoss_
                            ak = obj.ATuning / (obj.K_ + 1 + obj.ACommon)^obj.Alpha;
                            ghat = (obj.LossPlus_ - lossMeasurement) ./ (2*obj.Ck_*obj.Delta_);
                            obj.Theta_ = obj.Theta_ - ak*ghat;
                            obj.K_ = obj.K_ + 1;
                        end
                        obj.HasStoredLoss_ = false;
                        obj.Ck_ = obj.CTuning / (obj.K_ + 1)^obj.Gamma;
                        obj.Delta_ = 2*round(rand(n,1)) - 1;
                        paramToApply = obj.Theta_ + obj.Ck_ * obj.Delta_;
                        obj.Phase_ = 2;

                    case 2
                        obj.LossPlus_ = lossMeasurement;
                        obj.HasStoredLoss_ = true;
                        paramToApply = obj.Theta_ - obj.Ck_ * obj.Delta_;
                        obj.Phase_ = 1;
                end
            else
                % ---- 3-phase immediate cycle ----
                switch obj.Phase_
                    case 1
                        obj.Ck_ = obj.CTuning / (obj.K_ + 1)^obj.Gamma;
                        obj.Delta_ = 2*round(rand(n,1)) - 1;
                        paramToApply = obj.Theta_ + obj.Ck_ * obj.Delta_;
                        obj.Phase_ = 2;

                    case 2
                        obj.LossPlus_ = lossMeasurement;
                        paramToApply = obj.Theta_ - obj.Ck_ * obj.Delta_;
                        obj.Phase_ = 3;

                    otherwise
                        lossMinus = lossMeasurement;
                        ak = obj.ATuning / (obj.K_ + 1 + obj.ACommon)^obj.Alpha;
                        ghat = (obj.LossPlus_ - lossMinus) ./ (2*obj.Ck_*obj.Delta_);
                        obj.Theta_ = obj.Theta_ - ak*ghat;
                        obj.K_ = obj.K_ + 1;
                        obj.Phase_ = 1;
                        paramToApply = obj.Theta_;
                end
            end

            thetaEstimate = obj.Theta_;
        end

        function [sz1, sz2] = getOutputSizeImpl(obj)
            sz1 = [obj.NumParameters, 1];
            sz2 = [obj.NumParameters, 1];
        end

        function [dt1, dt2] = getOutputDataTypeImpl(~)
            dt1 = 'double'; dt2 = 'double';
        end

        function [cp1, cp2] = isOutputComplexImpl(~)
            cp1 = false; cp2 = false;
        end

        function [fz1, fz2] = isOutputFixedSizeImpl(~)
            fz1 = true; fz2 = true;
        end

        function num = getNumInputsImpl(~)
            num = 1;
        end

        function num = getNumOutputsImpl(~)
            num = 2;
        end
    end
end
