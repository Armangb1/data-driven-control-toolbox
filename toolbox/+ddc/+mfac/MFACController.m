classdef MFACController < matlab.System
    %MFACCONTROLLER Compact-Form Dynamic Linearization Model-Free Adaptive Control.
    %   Implements the CFDL-MFAC scheme of Hou & Jin, "Model Free Adaptive
    %   Control: Theory and Applications". The plant is represented, at
    %   each step, by the compact-form dynamic linearization
    %
    %       y(k+1) = y(k) + phi(k) * du(k)
    %
    %   where phi(k) is the (unknown, time-varying) pseudo-partial
    %   derivative (PPD), estimated online by a projection algorithm, and
    %   du(k) = u(k) - u(k-1).
    %
    %   PPD estimate (projection algorithm with reset):
    %       phi(k) = phi(k-1) + eta*du(k-1)/(mu + du(k-1)^2) * (dy(k) - phi(k-1)*du(k-1))
    %       reset phi(k) -> phi(1) if |phi(k)| <= epsilon, |du(k-1)| <= epsilon,
    %       or sign(phi(k)) ~= sign(phi(1))  (robustness safeguard)
    %
    %   Control law:
    %       du(k) = rho*phi(k)/(lambda + phi(k)^2) * (r(k+1) - y(k))
    %       u(k)  = u(k-1) + du(k)
    %
    %   No plant model is required -- only measured input/output data.
    %   Usable as a plain MATLAB object or as a Simulink "MATLAB System"
    %   block.
    %
    %   Example (MATLAB):
    %       ctrl = ddc.mfac.MFACController('PhiInit', 1);
    %       u = ctrl.step(y, rNext);
    %
    %   See also ddc.str.DirectSTRController, ddc.common.RLSEstimator.

    properties (Nontunable)
        PhiInit (1,1) double {mustBeReal} = 1     % initial PPD estimate phi(1)
    end

    properties
        Eta     (1,1) double {mustBeReal} = 1        % PPD step size, (0,2]
        Mu      (1,1) double {mustBePositive} = 1     % PPD weighting
        Rho     (1,1) double {mustBeReal} = 1         % control step size, (0,1]
        Lambda  (1,1) double {mustBePositive} = 1     % control weighting
        Epsilon (1,1) double {mustBePositive} = 1e-5  % reset threshold
    end

    properties (Access = private)
        Phi_
        PrevY_
        PrevU_
        PrevDeltaU_
        Initialized_
    end

    methods
        function obj = MFACController(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.Phi_ = obj.PhiInit;
            obj.PrevY_ = 0;
            obj.PrevU_ = 0;
            obj.PrevDeltaU_ = 0;
            obj.Initialized_ = false;
        end

        function resetImpl(obj)
            obj.Phi_ = obj.PhiInit;
            obj.PrevY_ = 0;
            obj.PrevU_ = 0;
            obj.PrevDeltaU_ = 0;
            obj.Initialized_ = false;
        end

        function [u, phiOut] = stepImpl(obj, y, rNext)
            if ~obj.Initialized_
                % First call: no prior sample to differentiate against.
                obj.PrevY_ = y;
                obj.Initialized_ = true;
            end

            dy = y - obj.PrevY_;
            du_prev = obj.PrevDeltaU_;

            phi = obj.Phi_ + (obj.Eta*du_prev/(obj.Mu + du_prev^2)) * (dy - obj.Phi_*du_prev);

            if abs(phi) <= obj.Epsilon || abs(du_prev) <= obj.Epsilon || ...
                    sign(phi) ~= sign(obj.PhiInit)
                phi = obj.PhiInit;
            end

            du = (obj.Rho*phi/(obj.Lambda + phi^2)) * (rNext - y);
            u = obj.PrevU_ + du;

            obj.Phi_ = phi;
            obj.PrevY_ = y;
            obj.PrevU_ = u;
            obj.PrevDeltaU_ = du;

            phiOut = phi;
        end

        function [sz1, sz2] = getOutputSizeImpl(~)
            sz1 = [1 1];
            sz2 = [1 1];
        end

        function [dt1, dt2] = getOutputDataTypeImpl(~)
            dt1 = 'double';
            dt2 = 'double';
        end

        function [cp1, cp2] = isOutputComplexImpl(~)
            cp1 = false; cp2 = false;
        end

        function [fz1, fz2] = isOutputFixedSizeImpl(~)
            fz1 = true; fz2 = true;
        end

        function num = getNumInputsImpl(~)
            num = 2;
        end

        function num = getNumOutputsImpl(~)
            num = 2;
        end
    end
end
