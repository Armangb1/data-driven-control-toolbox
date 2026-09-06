classdef CandidateControllerBank < matlab.System
    %CANDIDATECONTROLLERBANK Bank of candidate controllers.
    %   Computes, at each step, the control signal that each of a bank of
    %   candidate controllers would produce. Candidates can be any proper
    %   LTI controller via tf objects (continuous or discrete), or a plain
    %   numeric vector of proportional gains (wrapped internally as
    %   discrete-time proportional controllers).
    %
    %   Continuous-time controllers are auto-discretized in setupImpl
    %   using c2d with the specified positive SampleTime and
    %   DiscretizationMethod. Discrete-time controllers pass through
    %   unchanged.
    %
    %   Used together with ddc.ufc.UnfalsifiedSwitchingController, which
    %   selects which candidate's output is actually applied to the plant
    %   based on the unfalsified (fictitious-reference) cost.
    %
    %   Each candidate C_i is applied as u_i = C_i * (r - y) using IIR
    %   filtering with internal state per candidate.
    %
    %   Usable as a plain MATLAB object or as a Simulink "MATLAB System"
    %   block.
    %
    %   See also ddc.ufc.UnfalsifiedSwitchingController.

    properties (Nontunable)
        Controllers = [0.5; 1; 2]  % tf array or numeric gain vector, one per candidate
        SampleTime     (1,1) double {mustBePositive} = 1
        DiscretizationMethod (1,1) string {mustBeMember(DiscretizationMethod, ...
            ["zoh","foh","tustin","matched","impulse"])} = "zoh"
    end

    properties (Access = private)
        BCoeffs_
        ACoeffs_
        UCandPast_
        MaxN_
    end

    methods
        function obj = CandidateControllerBank(varargin)
            setProperties(obj, nargin, varargin{:});
            obj.resolveControllers();
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.resolveControllers();
            n = numel(obj.Controllers);
            obj.BCoeffs_ = cell(n, 1);
            obj.ACoeffs_ = cell(n, 1);
            obj.MaxN_ = 0;
            for i = 1:n
                C = obj.Controllers(i);
                if C.Ts == 0
                    C = c2d(C, obj.SampleTime, char(obj.DiscretizationMethod));
                end
                [b, a] = tfdata(C, 'v');
                obj.BCoeffs_{i} = b(:)';
                obj.ACoeffs_{i} = a(:)';
                obj.MaxN_ = max(obj.MaxN_, numel(a) - 1);
            end
            nCol = max(obj.MaxN_, 1) + (obj.MaxN_ > 0);
            obj.UCandPast_ = zeros(n, nCol);
        end

        function resetImpl(obj)
            n = numel(obj.Controllers);
            nCol = max(obj.MaxN_, 1) + (obj.MaxN_ > 0);
            obj.UCandPast_ = zeros(n, nCol);
        end

        function uCandidates = stepImpl(obj, r, y)
            n = numel(obj.Controllers);
            e = r - y;
            uCandidates = zeros(n, 1);

            for i = 1:n
                uCandidates(i) = iirFilter(obj.BCoeffs_{i}, obj.ACoeffs_{i}, ...
                    e, obj.UCandPast_(i, :));
            end

            for i = 1:n
                obj.UCandPast_(i, :) = shiftAndInsert(obj.UCandPast_(i, :), ...
                    uCandidates(i), numel(obj.ACoeffs_{i}) - 1);
            end
        end

        function sz = getOutputSizeImpl(obj)
            sz = [obj.candidateCount(), 1];
        end

        function dt = getOutputDataTypeImpl(~)
            dt = 'double';
        end

        function cp = isOutputComplexImpl(~)
            cp = false;
        end

        function fz = isOutputFixedSizeImpl(~)
            fz = true;
        end

        function num = getNumInputsImpl(~)
            num = 2;
        end

        function num = getNumOutputsImpl(~)
            num = 1;
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

function uCand = iirFilter(b, a, e, uPast)
    M = numel(b) - 1;
    N = numel(a) - 1;
    uCand = b(1)*e;
    if M > 0
        uCand = uCand - b(2:end) * uPast(1:M)';
    end
    if N > 0
        uCand = uCand + a(2:end) * uPast(1:N)';
    end
end

function row = shiftAndInsert(row, val, n)
    if n > 0
        row(2:n+1) = row(1:n);
    end
    row(1) = val;
end
