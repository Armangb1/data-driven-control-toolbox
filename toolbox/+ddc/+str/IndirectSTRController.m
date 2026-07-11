classdef IndirectSTRController < matlab.System
    %INDIRECTSTRCONTROLLER Indirect self-tuning regulator (pole placement).
    %   Baseline adaptive-control scheme following Astrom & Wittenmark,
    %   "Adaptive Control", Ch. 5: identify the plant, then compute the
    %   controller from the plant estimate ("indirect" = identify-then-design).
    %
    %   Plant assumption (first-order ARX, unit delay):
    %       y(k) = -a1*y(k-1) + b1*u(k-1) + e(k)
    %   estimated online via RLS on regressor phi(k-1) = [y(k-1); u(k-1)].
    %
    %   Minimal-degree pole-placement RST design (R=1, S=s0, T=t0) places
    %   the closed loop at  1 + Ac1*q^-1 = 0 :
    %       s0 = (Ac1 - a1_hat) / b1_hat
    %       t0 = (1 + Ac1) / b1_hat        (unity DC gain from r to y)
    %       u(k) = t0*r(k) - s0*y(k)
    %
    %   Extending this class to higher-order plants requires a general
    %   Diophantine-equation solver in place of the closed-form s0/t0
    %   above; this minimal first-order case is the standard textbook
    %   baseline used to benchmark data-driven schemes (DeePC/MFAC/UFC).
    %
    %   Usable as a plain MATLAB object or as a Simulink "MATLAB System"
    %   block.
    %
    %   See also ddc.str.DirectSTRController, ddc.common.RLSEstimator.

    properties
        Ac1 (1,1) double {mustBeReal, mustBeGreaterThan(Ac1,-1), ...
                           mustBeLessThan(Ac1,1)} = -0.5  % desired closed-loop pole coeff
        ForgettingFactor (1,1) double {mustBeGreaterThan(ForgettingFactor,0), ...
                                        mustBeLessThanOrEqual(ForgettingFactor,1)} = 0.98
        MinB1 (1,1) double {mustBePositive} = 1e-3  % guard against near-zero b1 estimate
        InitialS0 (1,1) double {mustBeReal} = 0  % initial controller gain (avoids u==0 deadlock)
        InitialT0 (1,1) double {mustBeReal} = 1  % initial feedforward gain (avoids u==0 deadlock)
    end

    properties (Access = private)
        RLS_
        PrevY_
        PrevU_
        S0_
        T0_
    end

    methods
        function obj = IndirectSTRController(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.RLS_ = ddc.common.RLSEstimator('NumParameters', 2, ...
                'ForgettingFactor', obj.ForgettingFactor);
            obj.PrevY_ = 0;
            obj.PrevU_ = 0;
            obj.S0_ = obj.InitialS0;
            obj.T0_ = obj.InitialT0;
        end

        function resetImpl(obj)
            reset(obj.RLS_);
            obj.PrevY_ = 0;
            obj.PrevU_ = 0;
            obj.S0_ = obj.InitialS0;
            obj.T0_ = obj.InitialT0;
        end

        function [u, theta] = stepImpl(obj, y, r)
            phi = [obj.PrevY_; obj.PrevU_];
            theta = obj.RLS_.step(phi, y);
            a1 = -theta(1);
            b1 = theta(2);

            if abs(b1) < obj.MinB1
                % Keep previous controller gains if b1 estimate is unreliable.
                s0 = obj.S0_;
                t0 = obj.T0_;
            else
                s0 = (obj.Ac1 - a1) / b1;
                t0 = (1 + obj.Ac1) / b1;
            end

            u = t0*r - s0*y;

            obj.PrevY_ = y;
            obj.PrevU_ = u;
            obj.S0_ = s0;
            obj.T0_ = t0;
        end

        function [sz1, sz2] = getOutputSizeImpl(~)
            sz1 = [1 1];
            sz2 = [2 1];
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
            num = 2;
        end

        function num = getNumOutputsImpl(~)
            num = 2;
        end
    end
end
