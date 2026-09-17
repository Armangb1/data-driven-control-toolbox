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
        % NumParameters Number of parameters
        NumParameters (1,1) double {mustBePositive, mustBeInteger} = 1

        % InitialTheta Initial parameter estimate
        InitialTheta (:,1) double = 0

        % ParameterUpdateMode Parameter update mode
        ParameterUpdateMode (1,1) string {mustBeMember(ParameterUpdateMode, ["immediate","deferred"])} = "immediate"

        % SampleTime Sample Time (-1 for inherited)
        SampleTime (1,1) double = -1
    end

    properties
        % ATuning Numerator of a_k (step-size gain)
        ATuning (1,1) double {mustBePositive} = 0.1

        % CTuning Numerator of c_k (perturbation size)
        CTuning (1,1) double {mustBePositive} = 0.1

        % ACommon Stability constant Abar
        ACommon (1,1) double {mustBeNonnegative} = 10

        % Alpha Decay exponent for a_k
        Alpha (1,1) double {mustBePositive} = 0.602

        % Gamma Decay exponent for c_k
        Gamma (1,1) double {mustBePositive} = 0.101

        % GradientClipLower Lower bound for the clipped gradient estimate
        GradientClipLower (1,1) double = -inf

        % GradientClipUpper Upper bound for the clipped gradient estimate
        GradientClipUpper (1,1) double = inf
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

    methods (Access = private)
        function g = clipGradient(obj, g)
            g = min(max(g, obj.GradientClipLower), obj.GradientClipUpper);
        end
    end

    methods (Access = protected)
        function validatePropertiesImpl(obj)
            if obj.GradientClipLower > obj.GradientClipUpper
                error('ddc:spsa:GradientClipBounds', ...
                    'GradientClipLower must be less than or equal to GradientClipUpper.');
            end
        end

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
                            ghat = obj.clipGradient((obj.LossPlus_ - lossMeasurement) ./ (2*obj.Ck_*obj.Delta_));
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
                        ghat = obj.clipGradient((obj.LossPlus_ - lossMinus) ./ (2*obj.Ck_*obj.Delta_));
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

        function sts = getSampleTimeImpl(obj)
            if obj.SampleTime == -1
                sts = createSampleTime(obj, 'Type', 'Inherited');
            else
                sts = createSampleTime(obj, 'Type', 'Discrete', ...
                    'SampleTime', obj.SampleTime);
            end
        end
    end

    methods (Static, Access = protected)
        function header = getHeaderImpl
            header = matlab.system.display.Header(mfilename('class'), ...
                'Title', 'SPSA Optimizer', ...
                'Text', [ ...
                'Simultaneous Perturbation Stochastic Approximation.' ...
                newline ...
                'Online, gradient-free optimizer that tunes a parameter ' ...
                'vector from noisy scalar loss measurements only.']);
        end

        function groups = getPropertyGroupsImpl
            % --- Tab 1: Main setup ---
            mainSection = matlab.system.display.Section(...
                'Title', 'SPSAOptimizer', ...
                'PropertyList', {'NumParameters', 'InitialTheta', ...
                'ParameterUpdateMode', 'SampleTime'});

            mainGroup = matlab.system.display.SectionGroup(...
                'Title', 'Main', ...
                'Sections', mainSection);

            % --- Tab 2: Tuning gains ---
            tuningSection = matlab.system.display.Section(...
                'Title', 'Gain Sequences', ...
                'PropertyList', {'ATuning', 'CTuning', 'ACommon', ...
                'Alpha', 'Gamma'});

            tuningGroup = matlab.system.display.SectionGroup(...
                'Title', 'Tuning Gains', ...
                'Sections', tuningSection);

            % --- Tab 3: Gradient clipping ---
            gradientSection = matlab.system.display.Section(...
                'Title', 'Gradient Clipping', ...
                'PropertyList', {'GradientClipLower', 'GradientClipUpper'});

            gradientGroup = matlab.system.display.SectionGroup(...
                'Title', 'Gradient Clipping', ...
                'Sections', gradientSection);

            groups = [mainGroup, tuningGroup, gradientGroup];
        end
    end
end