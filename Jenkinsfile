pipeline {
    agent any

    options {
        timestamps()
    }

    parameters {
        string(name: 'EXPERIMENTAL_SRC_BRANCH', defaultValue: '',  description: 'experimental_src branch (empty = same as pipeline branch, fallback to devel)')
        string(name: 'JASTERIX_BRANCH',         defaultValue: 'devel', description: 'jASTERIX branch')

        // Test tags (checkboxes)
        booleanParam(name: 'TAG_SYSTEM',          defaultValue: true, description: 'Tag: system')
        booleanParam(name: 'TAG_IMPORT',          defaultValue: true, description: 'Tag: import')
        booleanParam(name: 'TAG_CALCULATE',       defaultValue: true, description: 'Tag: calculate')
        booleanParam(name: 'TAG_EVAL',            defaultValue: true, description: 'Tag: eval')
        booleanParam(name: 'TAG_UI',              defaultValue: true, description: 'Tag: ui (all UI tests)')
        booleanParam(name: 'TAG_VIEWS',           defaultValue: true, description: 'Tag: views')
        booleanParam(name: 'TAG_TABLEVIEW',       defaultValue: true, description: 'Tag: tableview')
        booleanParam(name: 'TAG_HISTOGRAMVIEW',   defaultValue: true, description: 'Tag: histogramview')
        booleanParam(name: 'TAG_SCATTERPLOTVIEW', defaultValue: true, description: 'Tag: scatterplotview')
        booleanParam(name: 'TAG_GEOGRAPHICVIEW',  defaultValue: true, description: 'Tag: geographicview')
        booleanParam(name: 'TAG_ANALYZE',         defaultValue: true, description: 'Tag: analyze (Analyze Data Source, MLAT + ADS-B)')
        booleanParam(name: 'TAG_ARTAS_SPF',       defaultValue: true, description: 'Tag: artas_spf (ARTAS TRI import/association/display, at_20230422)')
        booleanParam(name: 'TAG_MLAT_RU',         defaultValue: true, description: 'Tag: mlat_ru (MLAT contributing receivers, loww_20260609)')

        // Build options
        booleanParam(name: 'CLEAN_BUILD',            defaultValue: false, description: 'Clean build (remove build_deb10 before building)')
        booleanParam(name: 'ASAN',                   defaultValue: false, description: 'Build with AddressSanitizer (slower; use to diagnose heap/memory corruption)')

        // Datasets (checkboxes)
        booleanParam(name: 'DATASET_05H',  defaultValue: true,  description: 'Dataset: at_20230422_05h (0.5h)')
        booleanParam(name: 'DATASET_2H',   defaultValue: true,  description: 'Dataset: at_20230422_2h (2h)')
        booleanParam(name: 'DATASET_LOWW', defaultValue: true,  description: 'Dataset: loww_20260609_4h (Vienna airport surface, 4h)')
    }

    environment {
        GITHUB_TOKEN    = credentials('github-pat')
        CI_DIR          = '/home/user/ci'
        TEST_DATA_PATH  = '/home/user/ci/test'
        DOCKER_IMAGE    = 'compass/build_deb10'
        DISPLAY             = ':0'
        COMPASS_EXTRA_ARGS  = '--no_highdpi -r'
        PYTHONUNBUFFERED    = '1'
    }

    stages {
        stage('Checkout') {
            steps {
                script {
                    // Resolve branch parameters
                    // experimental_src: explicit param > pipeline branch > devel
                    def expBranch = params.EXPERIMENTAL_SRC_BRANCH?.trim() ?: env.BRANCH_NAME
                    def jasterixBranch = params.JASTERIX_BRANCH?.trim() ?: 'devel'

                    echo "experimental_src branch: ${expBranch} (fallback: devel)"
                    echo "jASTERIX branch: ${jasterixBranch}"

                    // Fresh clone experimental_src: try expBranch, fallback to devel
                    sh 'rm -rf experimental_src'
                    sh "git clone --depth 1 --branch ${expBranch} https://${GITHUB_TOKEN}@github.com/hpuhr/experimental_src.git experimental_src || git clone --depth 1 --branch devel https://${GITHUB_TOKEN}@github.com/hpuhr/experimental_src.git experimental_src"
                    // Fresh clone jASTERIX into the branch workspace - a shared
                    // sibling dir races with concurrent branch builds (rm -rf
                    // while another build compiles in it)
                    sh "rm -rf jasterix && git clone --depth 1 --branch ${jasterixBranch} https://github.com/hpuhr/jASTERIX.git jasterix"
                }
            }
        }

        stage('Build') {
            steps {
                script {
                    def cleanFlag = params.CLEAN_BUILD ? '--clean' : ''
                    def asanFlag  = params.ASAN ? '--asan' : ''
                    sh """
                        docker run --rm \
                            -v \$(pwd):/workspace/compass \
                            -v \$(pwd)/jasterix:/workspace/jasterix \
                            -w /workspace/compass/docker \
                            ${DOCKER_IMAGE} \
                            bash -c 'set -e; export WORKSPACE_BASE=/workspace; ./build_jasterix.sh ${cleanFlag} ${asanFlag} && ./build_compass.sh ${cleanFlag} ${asanFlag}'
                    """
                }
            }
        }

        stage('Unit Tests') {
            steps {
                script {
                    // When ASAN=true, LSan runs at process exit and returns its own
                    // nonzero exitcode (default 23) when it finds leaks - that would
                    // fail the stage even if all assertions passed. exitcode=0 keeps
                    // leak output visible while letting the real test exit code stand.
                    def lsanEnv = params.ASAN ? "LSAN_OPTIONS=exitcode=0 " : ''
                    sh """
                        docker run --rm --init \
                            -v \$(pwd):/workspace/compass \
                            -v \$(pwd)/jasterix:/workspace/jasterix \
                            -w /workspace/compass \
                            ${DOCKER_IMAGE} \
                            bash -c '${lsanEnv}MESA_GL_VERSION_OVERRIDE=3.3 MESA_GLSL_VERSION_OVERRIDE=330 xvfb-run -a ./build_deb10/bin/compass_tests'
                    """
                }
            }
        }

        stage('AppImage') {
            steps {
                script {
                    def asanFlag = params.ASAN ? '--asan' : ''
                    sh """
                        docker run --rm \
                            -v \$(pwd):/workspace/compass \
                            -v \$(pwd)/jasterix:/workspace/jasterix \
                            -v ${CI_DIR}:${CI_DIR} \
                            -w /workspace/compass \
                            ${DOCKER_IMAGE} \
                            bash -c 'set -e; export WORKSPACE_BASE=/workspace; sudo make -C /workspace/jasterix/build_deb10 install && sudo make -C /workspace/compass/build_deb10 install && cd /workspace/compass/docker && ./deploy_compass.sh ${asanFlag}'
                    """
                    // Collect artifacts
                    sh "bash docker/collect_artifacts.sh \$(pwd) ${BUILD_NUMBER} ${BRANCH_NAME}"
                }
            }
        }

        stage('Integration Tests') {
            when {
                expression {
                    def anyTag = params.TAG_SYSTEM || params.TAG_IMPORT || params.TAG_CALCULATE || params.TAG_EVAL ||
                                 params.TAG_UI || params.TAG_VIEWS || params.TAG_TABLEVIEW ||
                                 params.TAG_HISTOGRAMVIEW || params.TAG_SCATTERPLOTVIEW || params.TAG_GEOGRAPHICVIEW ||
                                 params.TAG_ANALYZE || params.TAG_ARTAS_SPF || params.TAG_MLAT_RU
                    def anyDataset = params.DATASET_05H || params.DATASET_2H || params.DATASET_LOWW
                    return anyTag && anyDataset
                }
            }
            steps {
                script {
                    // Build tags string from checkboxes
                    def tags = []
                    if (params.TAG_SYSTEM)          tags << 'system'
                    if (params.TAG_IMPORT)          tags << 'import'
                    if (params.TAG_CALCULATE)       tags << 'calculate'
                    if (params.TAG_EVAL)            { tags << 'eval'; tags << 'eval_loww' }
                    if (params.TAG_UI)              tags << 'ui'
                    if (params.TAG_VIEWS)           tags << 'views'
                    if (params.TAG_TABLEVIEW)       tags << 'tableview'
                    if (params.TAG_HISTOGRAMVIEW)   tags << 'histogramview'
                    if (params.TAG_SCATTERPLOTVIEW) tags << 'scatterplotview'
                    if (params.TAG_GEOGRAPHICVIEW)  tags << 'geographicview'
                    if (params.TAG_ANALYZE)         tags << 'analyze'
                    if (params.TAG_ARTAS_SPF)       tags << 'artas_spf'
                    if (params.TAG_MLAT_RU)         tags << 'mlat_ru'
                    def tagsStr = tags.join(',')

                    // Build dataset list from checkboxes: name (used for the log file
                    // name) + manifest path. Each manifest declares the tags its
                    // dataset supports; unsupported selected tags simply resolve to
                    // no targets for that dataset.
                    def datasets = []
                    if (params.DATASET_05H)  datasets << [name: 'at_20230422_05h',  manifest: "${TEST_DATA_PATH}/at_20230422/at_20230422_05h.json"]
                    if (params.DATASET_2H)   datasets << [name: 'at_20230422_2h',   manifest: "${TEST_DATA_PATH}/at_20230422/at_20230422_2h.json"]
                    if (params.DATASET_LOWW) datasets << [name: 'loww_20260609_4h', manifest: "${TEST_DATA_PATH}/loww_20260609/loww_20260609_4h.json"]

                    // Find the run directory created by collect_artifacts.sh
                    def runDir = sh(
                        script: "ls -dt ${CI_DIR}/*-${BUILD_NUMBER}-${BRANCH_NAME} 2>/dev/null | head -1",
                        returnStdout: true
                    ).trim()

                    if (!runDir) {
                        error "No artifact directory found for build ${BUILD_NUMBER}"
                    }

                    def appimage = "${runDir}/COMPASS_deb10-x86_64.AppImage"
                    def scriptsDir = "${runDir}/scripts"

                    // Run tests for each selected dataset
                    // When ASAN=true, set runtime options for the sanitized binary:
                    //   abort_on_error=1 makes ASan SIGABRT on first error so the
                    //     test framework's shutdown-crash flagging catches it.
                    //   log_path writes per-process ASan reports to <runDir>/asan.<pid>,
                    //     surviving pipe drops and archived with the build artifacts.
                    // LSAN_OPTIONS exitcode=0 keeps leak output visible but prevents
                    // LSan's default exit 23 (on detected leaks) from surfacing as a
                    // returncode - without this, isCrashed() in the test framework
                    // wrongly flags every clean shutdown with framework leaks as a crash.
                    // Real ASan errors (heap-corruption etc.) still abort via
                    // abort_on_error=1 → SIGABRT → watchdog relays 128+6=134.
                    // COMPASS_QUIT_TIMEOUT_SEC=180: ASan's atexit leak scan adds
                    // 10-30s to shutdown; the default 60s in closeCOMPASS would
                    // force-kill slow shutdowns, spuriously flagging tests as crashed.
                    def asanEnv = params.ASAN ? "ASAN_OPTIONS='abort_on_error=1:log_path=${runDir}/asan' LSAN_OPTIONS='exitcode=0' ASAN_SYMBOLIZER_PATH=/usr/bin/llvm-symbolizer COMPASS_QUIT_TIMEOUT_SEC=180 " : ''

                    def testFailures = []

                    for (dataset in datasets) {
                        echo "Running tests for dataset: ${dataset.name} (tags: ${tagsStr})${params.ASAN ? ' [ASan]' : ''}"

                        // The exit code of test_suite.py has to survive the `tee`:
                        // a shell pipeline reports the status of its LAST command,
                        // so without this the suite could crash and the step would
                        // still report success. `pipefail` is not used since it is
                        // not available in every /bin/sh - instead the status is
                        // written to a file inside the pipeline and re-raised after
                        // it. Streaming through tee is kept so the Jenkins console
                        // shows test progress live.
                        // returnStatus keeps the loop going: every dataset runs even
                        // if an earlier one failed, the stage is failed afterwards.
                        def rcFile = "${runDir}/test_${dataset.name}.rc"
                        def rc = sh(returnStatus: true, script: """
                            set +e
                            cd '${scriptsDir}/test_infra' || exit 2
                            { ${asanEnv}PYTHONPATH='${scriptsDir}' python3 test_suite.py \
                                --binary='${appimage}' \
                                --path='${scriptsDir}/tests' \
                                --manifest='${dataset.manifest}' \
                                --output='${TEST_DATA_PATH}' \
                                --tags='${tagsStr}' \
                                --deps=tests \
                                --no-prompt \
                                --cfg-override=none ; \
                              echo \$? > '${rcFile}' ; } \
                                2>&1 | tee '${runDir}/test_${dataset.name}.log'
                            exit `cat '${rcFile}'`
                        """)

                        // 0 = all tests passed, 1 = tests failed, 2 = suite could
                        // not be run (see ExitCode* in test_suite.py)
                        if (rc != 0) {
                            def reason = (rc == 1) ? 'test failures' : "suite error (exit ${rc})"
                            echo "Dataset ${dataset.name}: ${reason}"
                            testFailures << "${dataset.name} (${reason})"
                        }
                    }

                    // Fail the stage here, after all datasets have run, so the
                    // Stage View shows the failure on Integration Tests instead of
                    // on the post actions where the junit step publishes results.
                    if (testFailures) {
                        error "Integration tests failed: ${testFailures.join(', ')}"
                    }
                }
            }
        }
    }

    post {
        always {
            // Crash logs are kept on the host under
            // ${CI_DIR}/*-${BUILD_NUMBER}-${BRANCH_NAME}/compass_crash_*.log
            // so they're available per-build via SSH but don't clutter the
            // Jenkins artifact list. Workspace cleanup of any leftover copies
            // from earlier pipeline versions that did archive them.
            sh "rm -f compass_crash_*.log"
            // Archive test logs
            archiveArtifacts artifacts: '**/test_*.log', allowEmptyArchive: true
            // Copy JUnit XML into workspace and publish per-test results
            sh "cp ${TEST_DATA_PATH}/results/junit_results.xml junit_results.xml || true"
            junit testResults: 'junit_results.xml', allowEmptyResults: true
        }
        failure {
            echo 'Build or tests failed!'
        }
        success {
            echo 'All stages completed successfully.'
        }
    }
}
