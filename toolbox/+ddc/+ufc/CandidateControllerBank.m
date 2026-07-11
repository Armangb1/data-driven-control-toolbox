classdef CandidateControllerBank < matlab.System
    %CANDIDATECONTROLLERBANK Bank of proportional candidate controllers.
    %   Computes, at each step, the control signal that each of a bank of
    %   simple proportional candidate controllers u_i = Ki*(r - y) would
    %   produce. Used together with ddc.ufc.UnfalsifiedSwitchingController,
    %   which selects which candidate's output is actually applied to the
    %   plant based on the unfalsified (fictitious-reference) cost of
    %   each candidate.
    %
    %   Only proportional candidates are supported here because their
    %   fictitious-reference inverse is algebraic (r_hat = u/K + y); see
    %   ddc.ufc.MultimodelSwitchingController for a self-contained bank
    %   of dynamic (PI) candidates with a recursive fictitious-reference
    %   inverse.
    %
    %   Usable as a plain MATLAB object or as a Simulink "MATLAB System"
    %   block.
    %
    %   See also ddc.ufc.UnfalsifiedSwitchingController.

    properties (Nontunable)
        Gains (:,1) double {mustBeReal} = [0.5; 1; 2]  % candidate proportional gains
    end

    methods
        function obj = CandidateControllerBank(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function uCandidates = stepImpl(obj, r, y)
            uCandidates = obj.Gains * (r - y);
        end

        function sz = getOutputSizeImpl(obj)
            sz = [numel(obj.Gains), 1];
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
end
