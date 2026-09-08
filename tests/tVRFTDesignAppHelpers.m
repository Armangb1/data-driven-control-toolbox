classdef tVRFTDesignAppHelpers < matlab.unittest.TestCase
    %TVRFTDESIGNAPPHELPERS Tests for the ddc.vrft.VRFTDesignApp computational
    %helper functions (importTimeSeriesData, referenceModelFromSpec,
    %controllerBasisPreset, parseBasisExpression, assembleController,
    %runVrftDesign). No UI code is exercised here.

    methods (TestClassSetup)
        function addToolboxPath(testCase)
            here = fileparts(mfilename('fullpath'));
            toolboxPath = fullfile(here, '..', 'toolbox');
            addpath(toolboxPath);
            testCase.addTeardown(@() rmpath(toolboxPath));
        end
    end

    methods (Test)
        %% importTimeSeriesData
        function testImportFromWorkspaceArrays(testCase)
            source.Type = "workspace";
            source.U = (1:10)';
            source.Y = (11:20)';
            data = ddc.vrft.importTimeSeriesData(source);
            testCase.verifyEqual(data.U, (1:10)');
            testCase.verifyEqual(data.Y, (11:20)');
            testCase.verifyEqual(data.NumSamples, 10);
        end

        function testImportFromWorkspaceAcceptsRowVectors(testCase)
            source.Type = "workspace";
            source.U = 1:5;
            source.Y = 6:10;
            data = ddc.vrft.importTimeSeriesData(source);
            testCase.verifySize(data.U, [5 1]);
            testCase.verifySize(data.Y, [5 1]);
        end

        function testImportLengthMismatchErrors(testCase)
            source.Type = "workspace";
            source.U = 1:5;
            source.Y = 1:6;
            testCase.verifyError(@() ddc.vrft.importTimeSeriesData(source), ...
                'ddc:vrft:importTimeSeriesData:LengthMismatch');
        end

        function testImportUnknownTypeErrors(testCase)
            source.Type = "bogus";
            testCase.verifyError(@() ddc.vrft.importTimeSeriesData(source), ...
                'ddc:vrft:importTimeSeriesData:UnknownType');
        end

        function testImportFromMatFile(testCase)
            tmpDir = string(tempname);
            mkdir(tmpDir);
            testCase.addTeardown(@() rmdir(tmpDir, 's'));
            matFile = fullfile(tmpDir, "data.mat");
            u = (1:8)';
            y = (2:9)';
            save(matFile, 'u', 'y');

            source.Type = "mat";
            source.FilePath = matFile;
            source.UVariable = "u";
            source.YVariable = "y";
            data = ddc.vrft.importTimeSeriesData(source);
            testCase.verifyEqual(data.U, (1:8)');
            testCase.verifyEqual(data.Y, (2:9)');
        end

        function testImportFromMatFileMissingFileErrors(testCase)
            source.Type = "mat";
            source.FilePath = "does_not_exist_12345.mat";
            source.UVariable = "u";
            source.YVariable = "y";
            testCase.verifyError(@() ddc.vrft.importTimeSeriesData(source), ...
                'ddc:vrft:importTimeSeriesData:FileNotFound');
        end

        function testImportFromCsvFile(testCase)
            tmpDir = string(tempname);
            mkdir(tmpDir);
            testCase.addTeardown(@() rmdir(tmpDir, 's'));
            csvFile = fullfile(tmpDir, "data.csv");
            writematrix([1 10; 2 20; 3 30; 4 40], csvFile);

            source.Type = "csv";
            source.FilePath = csvFile;
            data = ddc.vrft.importTimeSeriesData(source);
            testCase.verifyEqual(data.U, [1;2;3;4]);
            testCase.verifyEqual(data.Y, [10;20;30;40]);
        end

        function testImportFromCsvFileTooFewColumnsErrors(testCase)
            tmpDir = string(tempname);
            mkdir(tmpDir);
            testCase.addTeardown(@() rmdir(tmpDir, 's'));
            csvFile = fullfile(tmpDir, "data.csv");
            writematrix([1; 2; 3], csvFile);

            source.Type = "csv";
            source.FilePath = csvFile;
            testCase.verifyError(@() ddc.vrft.importTimeSeriesData(source), ...
                'ddc:vrft:importTimeSeriesData:TooFewColumns');
        end

        %% referenceModelFromSpec
        function testReferenceModelFromSpecIsDiscreteAndProper(testCase)
            spec.SettlingTime = 2;
            spec.DampingRatio = 0.8;
            spec.Ts = 0.1;
            Md = ddc.vrft.referenceModelFromSpec(spec);
            testCase.verifyEqual(Md.Ts, 0.1);
            testCase.verifyTrue(isstable(Md));
        end

        function testReferenceModelFromSpecUnityDcGain(testCase)
            spec.SettlingTime = 3;
            spec.DampingRatio = 1;
            spec.Ts = 0.05;
            Md = ddc.vrft.referenceModelFromSpec(spec);
            dcGain = evalfr(Md, 1); % z = 1 -> DC gain of discrete system
            testCase.verifyEqual(dcGain, 1, 'AbsTol', 1e-6);
        end

        function testReferenceModelFromSpecInvalidDampingErrors(testCase)
            spec.SettlingTime = 2;
            spec.DampingRatio = 1.5;
            spec.Ts = 0.1;
            testCase.verifyError(@() ddc.vrft.referenceModelFromSpec(spec), ...
                'ddc:vrft:referenceModelFromSpec:InvalidDampingRatio');
        end

        function testReferenceModelFromSpecMissingFieldErrors(testCase)
            spec.SettlingTime = 2;
            spec.Ts = 0.1;
            testCase.verifyError(@() ddc.vrft.referenceModelFromSpec(spec), ...
                'ddc:vrft:referenceModelFromSpec:MissingField');
        end

        %% controllerBasisPreset
        function testControllerBasisPresetPID(testCase)
            basis = ddc.vrft.controllerBasisPreset("PID", 0.1);
            testCase.verifySize(basis, [1 3]);
            for i = 1:numel(basis)
                testCase.verifyEqual(basis{i}.Ts, 0.1);
            end
        end

        function testControllerBasisPresetIntegrator(testCase)
            basis = ddc.vrft.controllerBasisPreset("Integrator", 0.2);
            testCase.verifySize(basis, [1 1]);
        end

        function testControllerBasisPresetGain(testCase)
            basis = ddc.vrft.controllerBasisPreset("gain", 0.1); % case-insensitive
            testCase.verifySize(basis, [1 1]);
            testCase.verifyEqual(basis{1}.Ts, 0.1);
        end

        function testControllerBasisPresetUnknownErrors(testCase)
            testCase.verifyError(@() ddc.vrft.controllerBasisPreset("bogus", 0.1), ...
                'ddc:vrft:controllerBasisPreset:UnknownPreset');
        end

        %% parseBasisExpression
        function testParseBasisExpressionIntegrator(testCase)
            Ts = 0.1;
            L = ddc.vrft.parseBasisExpression("Ts/(z-1)", Ts);
            expected = tf(Ts, [1 -1], Ts);
            testCase.verifyEqual(L.Numerator, expected.Numerator, 'AbsTol', 1e-12);
            testCase.verifyEqual(L.Denominator, expected.Denominator, 'AbsTol', 1e-12);
        end

        function testParseBasisExpressionScalar(testCase)
            Ts = 0.1;
            sys = ddc.vrft.parseBasisExpression("1", Ts);
            testCase.verifyEqual(sys.Ts, Ts);
            testCase.verifyEqual(sys.Numerator{1}, 1);
        end

        function testParseBasisExpressionDisallowedCharactersErrors(testCase)
            testCase.verifyError(@() ddc.vrft.parseBasisExpression("eval('bad')", 0.1), ...
                'ddc:vrft:parseBasisExpression:DisallowedCharacters');
        end

        function testParseBasisExpressionEmptyErrors(testCase)
            testCase.verifyError(@() ddc.vrft.parseBasisExpression("", 0.1), ...
                'ddc:vrft:parseBasisExpression:EmptyExpression');
        end

        %% assembleController
        function testAssembleControllerPI(testCase)
            Ts = 0.1;
            basis = ddc.vrft.controllerBasisPreset("PID", Ts);
            basis = basis(1:2); % {1, Ts/(z-1)}
            theta = [0.6; 1.2];
            C = ddc.vrft.assembleController(theta, basis);
            expected = 0.6 + 1.2 * tf(Ts, [1 -1], Ts);
            [numC, denC] = tfdata(C, 'v');
            [numE, denE] = tfdata(expected, 'v');
            testCase.verifyEqual(numC, numE, 'AbsTol', 1e-10);
            testCase.verifyEqual(denC, denE, 'AbsTol', 1e-10);
        end

        function testAssembleControllerSizeMismatchErrors(testCase)
            basis = ddc.vrft.controllerBasisPreset("PID", 0.1);
            theta = [1; 2]; % wrong size vs 3-entry basis
            testCase.verifyError(@() ddc.vrft.assembleController(theta, basis), ...
                'ddc:vrft:assembleController:SizeMismatch');
        end

        function testAssembleControllerEmptyBasisErrors(testCase)
            testCase.verifyError(@() ddc.vrft.assembleController([], {}), ...
                'ddc:vrft:assembleController:EmptyBasis');
        end

        %% runVrftDesign
        function testRunVrftDesignRecoversIdealController(testCase)
            rng(2);
            Ts = 0.1;
            plant = tf(0.5, [1 -0.8], Ts);
            Md = tf(0.3, [1 -0.7], Ts);
            basis = { tf(1,1,Ts), tf(Ts,[1 -1],Ts) };

            T = 500;
            u = 2*(rand(T,1) > 0.5) - 1;
            t = (0:T-1)'*Ts;
            y = lsim(plant, u, t);

            result = ddc.vrft.runVrftDesign(u, y, Md, basis, []);
            testCase.verifyTrue(result.Success);
            testCase.verifyEqual(result.Theta, [0.6; 1.2], 'AbsTol', 1e-3);
        end

        function testRunVrftDesignReportsLengthMismatchWithoutThrowing(testCase)
            Ts = 0.1;
            Md = tf(0.3, [1 -0.7], Ts);
            basis = { tf(1,1,Ts) };
            u = ones(10,1);
            y = ones(8,1);

            result = ddc.vrft.runVrftDesign(u, y, Md, basis, []);
            testCase.verifyFalse(result.Success);
            testCase.verifyEqual(result.ErrorId, 'ddc:vrft:vrftDesign:LengthMismatch');
        end

        function testRunVrftDesignAcceptsCustomPrefilter(testCase)
            rng(3);
            Ts = 0.1;
            plant = tf(0.5, [1 -0.8], Ts);
            Md = tf(0.3, [1 -0.7], Ts);
            basis = { tf(1,1,Ts), tf(Ts,[1 -1],Ts) };
            L = tf(1, 1, Ts); % trivial prefilter override

            T = 300;
            u = 2*(rand(T,1) > 0.5) - 1;
            t = (0:T-1)'*Ts;
            y = lsim(plant, u, t);

            result = ddc.vrft.runVrftDesign(u, y, Md, basis, L);
            testCase.verifyTrue(result.Success);
            testCase.verifyEqual(result.Info.L.Numerator, L.Numerator);
        end
    end
end
