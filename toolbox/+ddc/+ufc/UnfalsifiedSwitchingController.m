classdef UnfalsifiedSwitchingController < matlab.System
    %UNFALSIFIEDSWITCHINGCONTROLLER Unfalsified adaptive switching control.
    %   Implements the core unfalsified-control supervisory switching
    %   logic for a bank of proper discrete-time candidate controllers.
    %
    %   At each step the fictitious tracking error for candidate i is
    %   reconstructed recursively from the actually-applied control
    %   signal and the controller's difference equation:
    %
    %       u(k) = b0*e(k) + b1*e(k-1) + ... - a1*u(k-1) - ...
    %       ehat_i(k) = (1/b0)*[uActual(k)
    %                     - sum bj*ehat_i(k-j) + sum aj*uActual(k-j)]
    %
    %   A cost is accumulated per candidate with exponential forgetting:
    %       N_i(k) = lam*N_i(k-1) + W2*uCand_i(k)^2 + W1*ehat_i(k)^2
    %       D_i(k) = lam*D_i(k-1) + rtilde_i(k)^2
    %       V_i(k) = N_i(k) / (D_i(k) + eps)
    %   where lam is the ForgettingFactor, W1 weights the fictitious
    %   error, W2 weights the control effort, and rtilde_i = y - ehat_i.
    %   The controller switches to the lowest-cost candidate subject to
    %   a hysteresis margin.
    %
    %   The Controllers parameter accepts either a tf array or a plain
    %   numeric vector of proportional gains (wrapped internally as
    %   discrete-time proportional controllers). Continuous-time
    %   controllers are auto-discretized using c2d. The discretization
    %   sample time is set via the SampleTime property (must be positive).
    %
    %   Usable as a plain MATLAB object or as a Simulink "MATLAB System"
    %   block. Pair with ddc.ufc.CandidateControllerBank.
    %
    %   See also ddc.ufc.CandidateControllerBank, ddc.ufc.MultimodelSwitchingController.

    properties (Nontunable)
        Controllers = [0.5; 1; 2]  % tf array or numeric gain vector, one per candidate
        SampleTime     (1,1) double {mustBePositive} = 1
        DiscretizationMethod (1,1) string {mustBeMember(DiscretizationMethod, ...
            ["zoh","foh","tustin","matched","impulse"])} = "zoh"
    end

    properties
        ForgettingFactor (1,1) double {mustBeGreaterThan(ForgettingFactor,0), ...
                                        mustBeLessThanOrEqual(ForgettingFactor,1)} = 0.95
        HysteresisMargin (1,1) double {mustBeNonnegative} = 1e-3
        Epsilon          (1,1) double {mustBePositive} = 1e-12
        W1               (1,1) double {mustBeNonnegative} = 1  % weight of ehat in cost
        W2               (1,1) double {mustBeNonnegative} = 1  % weight of uCand in cost
    end

    properties (Access = private)
        V_
        N_
        D_
        ActiveIndex_
        BCoeffs_
        ACoeffs_
        EhatPast_
        UActualPast_
        UActualPrev_
        MaxM_
        MaxN_
        Ts_
    end

    methods
        function obj = UnfalsifiedSwitchingController(varargin)
            setProperties(obj, nargin, varargin{:});
            obj.resolveControllers();
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.resolveControllers();
            n = numel(obj.Controllers);
            obj.Ts_ = obj.SampleTime;
            obj.V_ = zeros(n, 1);
            obj.N_ = zeros(n, 1);
            obj.D_ = zeros(n, 1);
            obj.ActiveIndex_ = 1;

            obj.BCoeffs_ = cell(n, 1);
            obj.ACoeffs_ = cell(n, 1);
            obj.MaxM_ = 0;
            obj.MaxN_ = 0;
            for i = 1:n
                C = obj.Controllers(i);
                if C.Ts == 0
                    C = c2d(C, obj.Ts_, char(obj.DiscretizationMethod));
                end
                [b, a] = tfdata(C, 'v');
                obj.BCoeffs_{i} = b(:)';
                obj.ACoeffs_{i} = a(:)';
                obj.MaxM_ = max(obj.MaxM_, numel(b) - 1);
                obj.MaxN_ = max(obj.MaxN_, numel(a) - 1);
            end

            m = max(obj.MaxM_, 1);
            nAct = max(obj.MaxN_, 1);
            obj.EhatPast_ = zeros(n, m);
            obj.UActualPast_ = zeros(nAct, 1);
            obj.UActualPrev_ = 0;
        end

        function resetImpl(obj)
            n = numel(obj.Controllers);
            obj.V_ = zeros(n, 1);
            obj.N_ = zeros(n, 1);
            obj.D_ = zeros(n, 1);
            obj.ActiveIndex_ = 1;

            m = max(obj.MaxM_, 1);
            nAct = max(obj.MaxN_, 1);
            obj.EhatPast_ = zeros(n, m);
            obj.UActualPast_ = zeros(nAct, 1);
            obj.UActualPrev_ = 0;
        end

        function [uSelected, activeIndex, costs] = stepImpl(obj, uCandidates, y)
            n = numel(obj.Controllers);
            lam = obj.ForgettingFactor;

            ehat = computeFictitiousRefs(obj, uCandidates(obj.ActiveIndex_));

            obj.N_ = lam*obj.N_ + obj.W2*uCandidates.^2 + obj.W1*ehat.^2;
            obj.D_ = lam*obj.D_ + (y + ehat).^2;
            V_t = obj.N_ ./ (obj.D_ + obj.Epsilon);

            obj.V_ = max(obj.V_, V_t);
            obj.ActiveIndex_ = selectController(obj.V_, obj.ActiveIndex_, ...
                obj.HysteresisMargin);

            activeIndex = obj.ActiveIndex_;
            uSelected = uCandidates(activeIndex);
            costs = obj.V_;

            obj.EhatPast_ = shiftRows(obj.EhatPast_, ehat, obj.MaxM_);
            obj.UActualPast_ = shiftVec(obj.UActualPast_, uSelected, obj.MaxN_);
            obj.UActualPrev_ = uSelected;
        end

        function [sz1, sz2, sz3] = getOutputSizeImpl(obj)
            sz1 = [1 1];
            sz2 = [1 1];
            sz3 = [obj.candidateCount(), 1];
        end

        function [dt1, dt2, dt3] = getOutputDataTypeImpl(~)
            dt1 = 'double'; dt2 = 'double'; dt3 = 'double';
        end

        function [cp1, cp2, cp3] = isOutputComplexImpl(~)
            cp1 = false; cp2 = false; cp3 = false;
        end

        function [fz1, fz2, fz3] = isOutputFixedSizeImpl(~)
            fz1 = true; fz2 = true; fz3 = true;
        end

        function num = getNumInputsImpl(~)
            num = 2;
        end

        function num = getNumOutputsImpl(~)
            num = 3;
        end
    end

    methods (Access = protected)
        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', ...
                'SampleTime', obj.SampleTime);
        end
    end

    methods (Access = private)
        function resolveControllers(obj)
            if isempty(obj.Controllers)
                obj.Controllers = [0.5; 1; 2];
            end
            if isnumeric(obj.Controllers)
                gains = obj.Controllers(:);
                obj.Controllers = arrayfun(@(k) tf(k, 1, -1), gains, ...
                    'UniformOutput', false);
                obj.Controllers = [obj.Controllers{:}];
            end
        end

        function n = candidateCount(obj)
            n = max(numel(obj.Controllers), 1);
        end
    end
end

function ehat = computeFictitiousRefs(obj, uActual)
    n = numel(obj.BCoeffs_);
    ehat = zeros(n, 1);
    for i = 1:n
        b = obj.BCoeffs_{i};
        a = obj.ACoeffs_{i};
        M = numel(b) - 1;
        N = numel(a) - 1;

        num = uActual;
        if M > 0
            num = num - b(2:end) * obj.EhatPast_(i, 1:M)';
        end
        if N > 0
            num = num + a(2:end) * obj.UActualPast_(1:N);
        end
        ehat(i) = num / b(1);
    end
end

function idx = selectController(V, currentIdx, hysteresis)
    [minV, bestIdx] = min(V);
    if bestIdx ~= currentIdx && minV < V(currentIdx) - hysteresis
        idx = bestIdx;
    else
        idx = currentIdx;
    end
end

function mat = shiftRows(mat, vals, maxCols)
    if maxCols > 0
        mat(:, 2:maxCols) = mat(:, 1:maxCols-1);
    end
    mat(:, 1) = vals;
end

function vec = shiftVec(vec, val, maxN)
    if maxN > 0
        vec(2:maxN) = vec(1:maxN-1);
    end
    vec(1) = val;
end
