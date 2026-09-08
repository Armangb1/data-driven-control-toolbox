classdef MultimodelSwitchingController < matlab.System
    %MULTIMODELSWITCHINGCONTROLLER Multimodel unfalsified adaptive
    %supervisory switching control (MMUASSC).
    %
    %   Extension of ddc.ufc.UnfalsifiedSwitchingController to a bank of
    %   candidate PAIRS (C_i, M_i), where C_i is a candidate controller
    %   and M_i is a candidate reference MODEL of the plant. Each pair is
    %   scored by comparing the REAL signals against what the pair
    %   predicts, and the lowest-cost pair is switched into the real loop.
    %
    %   The candidate control values are NOT computed internally. They are
    %   supplied as an input uCandidates (a length-n vector), where
    %   uCandidates(i) is the control value that candidate controller C_i
    %   would apply this step (computed upstream, e.g. by
    %   ddc.ufc.CandidateControllerBank). The controller selected at the
    %   previous step (one-sample delay, as with a ZOH actuator) drives
    %   the actual plant, producing the real applied signal pair
    %   z(k) = [y(k); u(k)].
    %
    %   Two loops are involved at each step k:
    %
    %   (b) POTENTIAL loop: for each candidate i the fictitious reference
    %       rtilde_i(k) = ehat_i(k) + y(k) is reconstructed by solving
    %       C_i's difference equation backwards from the APPLIED control
    %       u(k) that this step produced:
    %
    %           u(k) = b0*e(k) + b1*e(k-1) + ... - a1*u(k-1) - ...
    %           ehat_i(k) = (1/b0)*[u(k)
    %                         - sum bj*ehat_i(k-j) + sum aj*u(k-j)]
    %
    %       with e = rtilde - y, i.e. rtilde_i(k) = ehat_i(k) + y(k).
    %
    %   (c) CANDIDATE loop: rtilde_i(k) is fed into the closed loop of C_i
    %       and the MODEL M_i (not the real plant):
    %
    %           e_i(k) = rtilde_i(k) - y_i(k)
    %           u_i(k) = C_i( e_i(k) )
    %           y_i(k) = M_i( u_i(k) )
    %
    %       Since C_i and M_i have direct feedthrough, this set of equations
    %       is circular and is resolved algebraically per sample using the
    %       past-state (difference-equation) contributions of C_i and M_i:
    %
    %           ucPast_i = recur. terms of C_i from past e_i, u_i
    %           ymPast_i = recur. terms of M_i from past u_i, y_i
    %           y_i(k) = (bM1*bC1*rtilde_i(k) + bM1*ucPast_i + ymPast_i)/(1 + bM1*bC1)
    %           u_i(k) = bC1*(rtilde_i(k) - y_i(k)) + ucPast_i
    %
    %       giving the predicted signal z_i(k) = [y_i(k); u_i(k)] from a
    %       genuine forward simulation with its own internal state per
    %       candidate (both C_i's and M_i's difference-equation states).
    %       When C_i/M_i exactly describe the real loop, z_i(k) reproduces
    %       z(k) = [y(k); u(k)] and the cost V_i stays at zero.
    %
    %   COST (corrected): the pair cost is the normalized prediction
    %   error accumulated with exponential forgetting and a running max,
    %
    %           N_i(k) = lam*N_i(k-1) + || z(k) - z_i(k) ||^2
    %           D_i(k) = lam*D_i(k-1) + || z_i(k) ||^2
    %           V_i(k) = max( V_i(k-1), N_i(k) / (D_i(k) + eps) )
    %
    %   i.e. V(C_i, z, t) = sup_tau || z(tau) - z_i(tau) || / ( || z_i(tau) || + eps ).
    %   The controller switches to the lowest-cost candidate subject to a
    %   hysteresis margin (the switch takes effect on the next sample).
    %   When C_i/M_i match the true plant, z_i reproduces z and V_i
    %   settles near zero.
    %
    %   The Controllers and Models parameters each accept a tf array or a
    %   plain numeric vector of gains (wrapped internally as discrete-time
    %   proportional terms). Continuous-time models are auto-discretized
    %   using c2d with the SampleTime and DiscretizationMethod properties.
    %
    %   Usable as a plain MATLAB object or as a Simulink "MATLAB System"
    %   block. Pair with ddc.ufc.CandidateControllerBank.
    %
    %   See also ddc.ufc.UnfalsifiedSwitchingController, ddc.ufc.CandidateControllerBank.

    properties (Nontunable)
        Controllers = [0.5; 1; 2]  % tf array or numeric gain vector, one per candidate
        Models      = [0.5; 1; 2]  % tf array or numeric gain vector, one per candidate
        SampleTime     (1,1) double {mustBePositive} = 1
        DiscretizationMethod (1,1) string {mustBeMember(DiscretizationMethod, ...
            ["zoh","foh","tustin","matched","impulse"])} = "zoh"
    end

    properties
        ForgettingFactor (1,1) double {mustBeGreaterThan(ForgettingFactor,0), ...
                                        mustBeLessThanOrEqual(ForgettingFactor,1)} = 0.95
        HysteresisMargin (1,1) double {mustBeNonnegative} = 1e-3
        Epsilon          (1,1) double {mustBePositive} = 1e-12
    end

    properties (Access = private)
        V_
        N_
        D_
        ActiveIndex_
        BCoeffs_     % controller numerator coefficients, one cell per candidate
        ACoeffs_     % controller denominator coefficients, one cell per candidate
        BMCoeffs_    % model numerator coefficients, one cell per candidate
        AMCoeffs_    % model denominator coefficients, one cell per candidate
        EhatPast_    % fictitious error history, size (n, MaxM_) -- loop (b)
        UActualPast_ % applied-control history, size (MaxN_, 1) -- loop (b)
        UActualPrev_ % last applied control u(k-1)
        EiPast_      % candidate-loop tracking-error history, size (n, MaxM_) -- loop (c)
        UiPast_      % candidate-loop control history, size (n, max(MaxN_,MaxMM_)) -- loop (c)
        YiPast_      % candidate-loop output history, size (n, MaxNM_) -- loop (c)
        MaxM_        % max controller numerator order
        MaxN_        % max controller denominator order
        MaxMM_       % max model numerator order
        MaxNM_       % max model denominator order
        Ts_
    end

    methods
        function obj = MultimodelSwitchingController(varargin)
            setProperties(obj, nargin, varargin{:});
            obj.resolveControllers();
            obj.resolveModels();
            if numel(obj.Controllers) ~= numel(obj.Models)
                error('ddc:ufc:MultimodelSwitchingController:PairSizeMismatch', ...
                    'Controllers and Models must contain the same number of candidates.');
            end
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.resolveControllers();
            obj.resolveModels();
            if numel(obj.Controllers) ~= numel(obj.Models)
                error('ddc:ufc:MultimodelSwitchingController:PairSizeMismatch', ...
                    'Controllers and Models must contain the same number of candidates.');
            end
            obj.Ts_ = obj.SampleTime;
            n = numel(obj.Controllers);

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

            obj.BMCoeffs_ = cell(n, 1);
            obj.AMCoeffs_ = cell(n, 1);
            obj.MaxMM_ = 0;
            obj.MaxNM_ = 0;
            for i = 1:n
                Md = obj.Models(i);
                if Md.Ts == 0
                    Md = c2d(Md, obj.Ts_, char(obj.DiscretizationMethod));
                end
                [b, a] = tfdata(Md, 'v');
                obj.BMCoeffs_{i} = b(:)';
                obj.AMCoeffs_{i} = a(:)';
                obj.MaxMM_ = max(obj.MaxMM_, numel(b) - 1);
                obj.MaxNM_ = max(obj.MaxNM_, numel(a) - 1);
            end

            mC = max(obj.MaxM_, 1);
            nC = max(obj.MaxN_, 1);
            nM = max(obj.MaxNM_, 1);
            nU = max(max(obj.MaxN_, obj.MaxMM_), 1);

            obj.EhatPast_ = zeros(n, mC);
            obj.UActualPast_ = zeros(nC, 1);
            obj.UActualPrev_ = 0;

            obj.EiPast_ = zeros(n, mC + (obj.MaxM_ > 0));
            obj.UiPast_ = zeros(n, nU + (max(obj.MaxN_, obj.MaxMM_) > 0));
            obj.YiPast_ = zeros(n, nM + (obj.MaxNM_ > 0));
        end

        function resetImpl(obj)
            n = numel(obj.Controllers);
            obj.V_ = zeros(n, 1);
            obj.N_ = zeros(n, 1);
            obj.D_ = zeros(n, 1);
            obj.ActiveIndex_ = 1;

            mC = max(obj.MaxM_, 1);
            nC = max(obj.MaxN_, 1);
            nM = max(obj.MaxNM_, 1);
            nU = max(max(obj.MaxN_, obj.MaxMM_), 1);

            obj.EhatPast_ = zeros(n, mC);
            obj.UActualPast_ = zeros(nC, 1);
            obj.UActualPrev_ = 0;

            obj.EiPast_ = zeros(n, mC + (obj.MaxM_ > 0));
            obj.UiPast_ = zeros(n, nU + (max(obj.MaxN_, obj.MaxMM_) > 0));
            obj.YiPast_ = zeros(n, nM + (obj.MaxNM_ > 0));
        end

        function [uSelected, activeIndex, costs] = stepImpl(obj, uCandidates, y)
            n = numel(obj.Controllers);
            lam = obj.ForgettingFactor;

            % ---- Select from the externally-provided candidate bank ----
            uSelected = uCandidates(obj.ActiveIndex_);

            % ---- Loop (b): fictitious references from the APPLIED control.
            % The real signal pair this step is z(k) = [y(k); u(k)] with
            % u(k) = uSelected; reconstruct rtilde_i(k) = ehat_i(k) + y(k)
            % by solving C_i's difference equation backwards from u(k).
            ehat = computeFictitiousRefs(obj, uSelected);
            rtilde = ehat + y;

            % ---- Loop (c): forward-simulate C_i/M_i closed loop ----
            % The loop e_i = rtilde_i - y_i, u_i = C_i(e_i), y_i = M_i(u_i)
            % is circular when C_i and M_i have direct feedthrough; it is
            % resolved algebraically per sample using each transfer
            % function's past (state) contribution:
            %     ucPast_i = past-state terms of C_i
            %     ymPast_i = past-state terms of M_i
            %     y_i(k) = (bM1*bC1*rtilde_i(k) + bM1*ucPast_i + ymPast_i)/(1+bM1*bC1)
            %     u_i(k) = bC1*(rtilde_i(k)-y_i(k)) + ucPast_i
            uCand = zeros(n, 1);
            yCand = zeros(n, 1);
            eCand = zeros(n, 1);
            for i = 1:n
                bC1 = obj.BCoeffs_{i}(1);
                bM1 = obj.BMCoeffs_{i}(1);
                ucPast = pastContrib(obj.BCoeffs_{i}, obj.ACoeffs_{i}, ...
                    obj.EiPast_(i, 1:obj.MaxM_), obj.UiPast_(i, 1:obj.MaxN_));
                ymPast = pastContrib(obj.BMCoeffs_{i}, obj.AMCoeffs_{i}, ...
                    obj.UiPast_(i, 1:obj.MaxMM_), obj.YiPast_(i, 1:obj.MaxNM_));
                yCand(i) = (bM1*bC1*rtilde(i) + bM1*ucPast + ymPast) / (1 + bM1*bC1);
                eCand(i) = rtilde(i) - yCand(i);
                uCand(i) = bC1*eCand(i) + ucPast;
            end

            % ---- Cost: compare real z(k) vs predicted z_i(k) ----
            z = [y; uSelected];
            for i = 1:n
                zi = [yCand(i); uCand(i)];
                d = z - zi;
                obj.N_(i) = lam*obj.N_(i) + d'*d;
                obj.D_(i) = lam*obj.D_(i) + zi'*zi;
                obj.V_(i) = max(obj.V_(i), obj.N_(i) / (obj.D_(i) + obj.Epsilon));
            end

            % ---- Switching with hysteresis (selects for the next step) ----
            [minV, bestIdx] = min(obj.V_);
            if bestIdx ~= obj.ActiveIndex_ && ...
                    minV < obj.V_(obj.ActiveIndex_) - obj.HysteresisMargin
                obj.ActiveIndex_ = bestIdx;
            end

            activeIndex = obj.ActiveIndex_;
            costs = obj.V_;

            % ---- Update loop (b) memories ----
            obj.EhatPast_ = shiftRows(obj.EhatPast_, ehat, obj.MaxM_);
            obj.UActualPast_ = shiftVec(obj.UActualPast_, uSelected, obj.MaxN_);
            obj.UActualPrev_ = uSelected;

            % ---- Update candidate-loop memories (loop c) ----
            for i = 1:n
                obj.EiPast_(i, :) = shiftAndInsert(obj.EiPast_(i, :), eCand(i), obj.MaxM_);
                obj.UiPast_(i, :) = shiftAndInsert(obj.UiPast_(i, :), uCand(i), ...
                    max(obj.MaxN_, obj.MaxMM_));
                obj.YiPast_(i, :) = shiftAndInsert(obj.YiPast_(i, :), yCand(i), obj.MaxNM_);
            end
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

        function resolveModels(obj)
            if isempty(obj.Models)
                obj.Models = [0.5; 1; 2];
            end
            if isnumeric(obj.Models)
                gains = obj.Models(:);
                obj.Models = arrayfun(@(k) tf(k, 1, -1), gains, ...
                    'UniformOutput', false);
                obj.Models = [obj.Models{:}];
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

function pc = pastContrib(b, a, xPast, yPast)
    % Past-state contribution of filter b/a given current-sample history
    % xPast(j) = x(k-j) and yPast(j) = y(k-j):
    %     sum_{j>=1} b(j+1)*x(k-j) - sum_{j>=1} a(j+1)*y(k-j)
    M = numel(b) - 1;
    N = numel(a) - 1;
    pc = 0;
    if M > 0
        pc = pc + b(2:end) * xPast(1:M)';
    end
    if N > 0
        pc = pc - a(2:end) * yPast(1:N)';
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

function row = shiftAndInsert(row, val, n)
    if n > 0
        row(2:n+1) = row(1:n);
    end
    row(1) = val;
end