classdef UnfalsifiedSwitchingController < matlab.System
    %UNFALSIFIEDSWITCHINGCONTROLLER Unfalsified adaptive switching control.
    %   Implements the core unfalsified-control supervisory switching
    %   logic of Safonov & Tsao, "The unfalsified control concept and
    %   learning", for a bank of proportional candidate controllers
    %   (produced externally by ddc.ufc.CandidateControllerBank).
    %
    %   At each step, the "fictitious reference" that WOULD have produced
    %   the actually-applied control/output pair, had candidate i been in
    %   the loop, is computed algebraically (valid for proportional
    %   candidates u_i = Ki*(r - y)):
    %
    %       ehat_i(k) = rhat_i(k) - y(k) = uActual(k-1) / Ki
    %
    %   A cost is accumulated per candidate with exponential forgetting,
    %   V_i(k) = lambda*V_i(k-1) + (1-lambda)*ehat_i(k)^2, and the
    %   controller switches to the lowest-cost candidate subject to a
    %   hysteresis margin and a minimum dwell time (to avoid chattering).
    %   This block remembers its own previous output internally, so no
    %   external feedback/delay wiring is required.
    %
    %   Usable as a plain MATLAB object or as a Simulink "MATLAB System"
    %   block. Pair with ddc.ufc.CandidateControllerBank (same Gains).
    %
    %   See also ddc.ufc.CandidateControllerBank, ddc.ufc.MultimodelSwitchingController.

    properties (Nontunable)
        Gains (:,1) double {mustBeReal} = [0.5; 1; 2]  % must match CandidateControllerBank.Gains
    end

    properties
        ForgettingFactor (1,1) double {mustBeGreaterThan(ForgettingFactor,0), ...
                                        mustBeLessThanOrEqual(ForgettingFactor,1)} = 0.95
        HysteresisMargin (1,1) double {mustBeNonnegative} = 1e-3
        MinDwellSteps    (1,1) double {mustBeNonnegative, mustBeInteger} = 5
        ResetCostOnSwitch (1,1) logical = true  % reset V on switch so new candidate starts fresh
    end

    properties (Access = private)
        V_
        ActiveIndex_
        DwellCounter_
        PrevUSelected_
    end

    methods
        function obj = UnfalsifiedSwitchingController(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            n = numel(obj.Gains);
            obj.V_ = zeros(n, 1);
            obj.ActiveIndex_ = 1;
            obj.DwellCounter_ = 0;
            obj.PrevUSelected_ = 0;
        end

        function resetImpl(obj)
            n = numel(obj.Gains);
            obj.V_ = zeros(n, 1);
            obj.ActiveIndex_ = 1;
            obj.DwellCounter_ = 0;
            obj.PrevUSelected_ = 0;
        end

        function [uSelected, activeIndex, costs] = stepImpl(obj, uCandidates)
            ehat = obj.PrevUSelected_ ./ obj.Gains;
            obj.V_ = obj.ForgettingFactor*obj.V_ + (1-obj.ForgettingFactor)*ehat.^2;

            [minV, bestIdx] = min(obj.V_);
            obj.DwellCounter_ = obj.DwellCounter_ + 1;
            if bestIdx ~= obj.ActiveIndex_ && obj.DwellCounter_ >= obj.MinDwellSteps && ...
                    minV < obj.V_(obj.ActiveIndex_) - obj.HysteresisMargin
                obj.ActiveIndex_ = bestIdx;
                obj.DwellCounter_ = 0;
                if obj.ResetCostOnSwitch
                    obj.V_ = zeros(size(obj.V_));
                end
            end

            activeIndex = obj.ActiveIndex_;
            uSelected = uCandidates(activeIndex);
            costs = obj.V_;

            obj.PrevUSelected_ = uSelected;
        end

        function [sz1, sz2, sz3] = getOutputSizeImpl(obj)
            sz1 = [1 1];
            sz2 = [1 1];
            sz3 = [numel(obj.Gains), 1];
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
            num = 1;
        end

        function num = getNumOutputsImpl(~)
            num = 3;
        end
    end
end
