classdef MultimodelSwitchingController < matlab.System
    %MULTIMODELSWITCHINGCONTROLLER Multimodel unfalsified adaptive switching control.
    %   Self-contained extension of ddc.ufc.UnfalsifiedSwitchingController
    %   to a bank of dynamic PI candidate controllers (rather than static
    %   proportional gains). Because PI candidates have internal state,
    %   this block internally maintains both the candidate control-signal
    %   generation and the switching/cost logic (i.e. it is the "masked
    %   subsystem" equivalent of a candidate bank + switching-logic pair,
    %   collapsed into a single MATLAB System block).
    %
    %   Each candidate is a discrete-time velocity-form PI controller:
    %       u_i(k) = u_i(k-1) + Kp_i*(e(k)-e(k-1)) + Ki_i*Ts*e(k),  e = r-y
    %
    %   The fictitious tracking error for candidate i is reconstructed
    %   recursively from the actually-applied control increment
    %   du(k) = uActual(k-1) - uActual(k-2):
    %
    %       ehat_i(k) = (du(k) + Kp_i*ehat_i(k-1)) / (Kp_i + Ki_i*Ts)
    %
    %   and accumulated into an exponentially-forgotten cost
    %   V_i(k) = lambda*V_i(k-1) + (1-lambda)*ehat_i(k)^2. The controller
    %   switches to the lowest-cost candidate subject to a hysteresis
    %   margin and minimum dwell time.
    %
    %   Usable as a plain MATLAB object or as a Simulink "MATLAB System"
    %   block.
    %
    %   See also ddc.ufc.UnfalsifiedSwitchingController.

    properties (Nontunable)
        KpGains (:,1) double {mustBeReal} = [0.5; 1; 2]     % candidate proportional gains
        KiGains (:,1) double {mustBeReal} = [0.1; 0.2; 0.5] % candidate integral gains
        SampleTime (1,1) double {mustBePositive} = 0.01
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
        EHatPrev_
        UCandPrev_
        EPrev_
        UPrev_
        UPrevPrev_
        ActiveIndex_
        DwellCounter_
    end

    methods
        function obj = MultimodelSwitchingController(varargin)
            setProperties(obj, nargin, varargin{:});
            if numel(obj.KpGains) ~= numel(obj.KiGains)
                error('ddc:ufc:MultimodelSwitchingController:GainSizeMismatch', ...
                    'KpGains and KiGains must have the same number of candidates.');
            end
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            n = numel(obj.KpGains);
            obj.V_ = zeros(n, 1);
            obj.EHatPrev_ = zeros(n, 1);
            obj.UCandPrev_ = zeros(n, 1);
            obj.EPrev_ = 0;
            obj.UPrev_ = 0;
            obj.UPrevPrev_ = 0;
            obj.ActiveIndex_ = 1;
            obj.DwellCounter_ = 0;
        end

        function resetImpl(obj)
            n = numel(obj.KpGains);
            obj.V_ = zeros(n, 1);
            obj.EHatPrev_ = zeros(n, 1);
            obj.UCandPrev_ = zeros(n, 1);
            obj.EPrev_ = 0;
            obj.UPrev_ = 0;
            obj.UPrevPrev_ = 0;
            obj.ActiveIndex_ = 1;
            obj.DwellCounter_ = 0;
        end

        function [uSelected, activeIndex, costs] = stepImpl(obj, r, y)
            e = r - y;
            Ts = obj.SampleTime;

            % 1) Reconstruct fictitious error per candidate from actual
            %    applied-control history, and update costs.
            du = obj.UPrev_ - obj.UPrevPrev_;
            denom = obj.KpGains + obj.KiGains*Ts;
            ehat = obj.EHatPrev_; % default: hold for degenerate candidates
            valid = abs(denom) > 1e-9;
            ehat(valid) = (du + obj.KpGains(valid).*obj.EHatPrev_(valid)) ./ denom(valid);
            obj.V_ = obj.ForgettingFactor*obj.V_ + (1-obj.ForgettingFactor)*ehat.^2;

            % 2) Candidate PI control outputs for this step.
            uCand = obj.UCandPrev_ + obj.KpGains*(e - obj.EPrev_) + obj.KiGains*Ts*e;

            % 3) Switching decision with hysteresis + dwell time.
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
            uSelected = uCand(activeIndex);
            costs = obj.V_;

            % 4) Update memories.
            obj.UPrevPrev_ = obj.UPrev_;
            obj.UPrev_ = uSelected;
            obj.EPrev_ = e;
            obj.UCandPrev_ = uCand;
            obj.EHatPrev_ = ehat;
        end

        function [sz1, sz2, sz3] = getOutputSizeImpl(obj)
            sz1 = [1 1];
            sz2 = [1 1];
            sz3 = [numel(obj.KpGains), 1];
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
end
