function results = runAllTests()
%RUNALLTESTS Run the full Data-Driven Control Toolbox test suite.
%   RESULTS = runAllTests() discovers and runs all matlab.unittest test
%   classes in this folder, adding the toolbox to the path first, and
%   prints a summary table. Exits with a nonzero status via ERROR if any
%   test fails, so it is CI-friendly.
%
%   Usage:
%       run('tests/runAllTests.m')

    here = fileparts(mfilename('fullpath'));
    toolboxPath = fullfile(here, '..', 'toolbox');
    addpath(toolboxPath);
    cleanupPath = onCleanup(@() rmpath(toolboxPath)); %#ok<NASGU>

    import matlab.unittest.TestSuite
    import matlab.unittest.TestRunner
    import matlab.unittest.plugins.TestReportPlugin

    suite = TestSuite.fromFolder(here, 'IncludingSubfolders', true);
    runner = TestRunner.withTextOutput('OutputDetail', 3);

    results = runner.run(suite);
    disp(table(results));

    numFailed = nnz([results.Failed]);
    if numFailed > 0
        error('ddc:tests:runAllTests:Failures', ...
            '%d of %d tests FAILED.', numFailed, numel(results));
    else
        fprintf('\nAll %d tests PASSED.\n', numel(results));
    end
end
