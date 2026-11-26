// Test tags of the checked boxes, as passed to test_suite.py --tags
def selectedTags() {
    def tags = []
    if (params.TAG_SYSTEM)          tags << 'system'
    if (params.TAG_IMPORT)          tags << 'import'
    if (params.TAG_CALCULATE)       tags << 'calculate'
    if (params.TAG_CONTEXT)         tags << 'context'
    if (params.TAG_CTX_GUI)         { tags << 'ctx_gui'; tags << 'ctx_gui_loww' }
    if (params.TAG_EVAL)            { tags << 'eval'; tags << 'eval_loww'; tags << 'eval_eddf' }
    if (params.TAG_UI)              tags << 'ui'
    if (params.TAG_VIEWS)           tags << 'views'
    if (params.TAG_TABLEVIEW)       tags << 'tableview'
    if (params.TAG_HISTOGRAMVIEW)   tags << 'histogramview'
    if (params.TAG_SCATTERPLOTVIEW) tags << 'scatterplotview'
    if (params.TAG_GEOGRAPHICVIEW)  tags << 'geographicview'
    if (params.TAG_ANALYZE)         { tags << 'analyze'; tags << 'analyze_smr' }
    if (params.TAG_ARTAS_SPF)       tags << 'artas_spf'
    if (params.TAG_MLAT_RU)         tags << 'mlat_ru'
    if (params.TAG_SENSOR_STATUS)   tags << 'sensor_status'
    if (params.TAG_TRACKER_CONTRIB) tags << 'tracker_contrib'
    if (params.TAG_LIVE)            tags << 'live'
    return tags
}

// Datasets of the checked boxes: name (names the output dir and the log file) + manifest
// path. Each manifest declares the tags its dataset supports; unsupported selected tags
// simply resolve to no targets for that dataset.
def selectedDatasets() {
    def path = env.TEST_DATA_PATH
    def datasets = []
    if (params.DATASET_05H)    datasets << [name: 'at_20230422_05h',  manifest: "${path}/at_20230422/at_20230422_05h.json"]
    if (params.DATASET_2H)     datasets << [name: 'at_20230422_2h',   manifest: "${path}/at_20230422/at_20230422_2h.json"]
    if (params.DATASET_LOWW)   datasets << [name: 'loww_20260609_4h', manifest: "${path}/loww_20260609/loww_20260609_4h.json"]
    if (params.DATASET_SKEYES) datasets << [name: 'skeyes_20251203',  manifest: "${path}/skeyes_20251203/skeyes_20251203.json"]
    if (params.DATASET_EDDF)   datasets << [name: 'eddf_20251117',    manifest: "${path}/eddf_20251117/eddf_20251117.json"]
    return datasets
}

def hasIntegrationSelection() {
    return !selectedTags().isEmpty() && !selectedDatasets().isEmpty()
}

// Runs one step of an OS build (build, unit tests, package, integration tests). A failure
// marks the stage and the build as failed but does not stop the pipeline: the remaining
// steps of this OS are skipped (osOk), the other OS continues. An abort stops the build.
def osStep(String os, Closure body) {
    catchError(buildResult: 'FAILURE', stageResult: 'FAILURE', catchInterruptions: false) {
        try {
            body()
        }
        catch (e) {
            env.FAILED_OS = ((env.FAILED_OS ?: '') + " ${os}").trim()
            throw e
        }
    }
}

// false once a step of the OS build failed (osStep)
def osOk(String os) {
    return !(env.FAILED_OS ?: '').tokenize(' ').contains(os)
}

// Shell command of one CI step in a build image container. Killing a `docker run`
// client (abort, Jenkins restart) does not stop the container - it keeps compiling in
// the workspace and collides with the next build. Every container is therefore labeled
// with branch and build, so it can be found and removed again (Checkout, post), and
// named so it is recognizable in `docker ps`. cmd runs in bash, without single quotes.
def dockerRun(String name, String image, String opts, String cmd) {
    return """docker run --rm --init \
        --name compass-${name}-${env.CI_TAG} \
        --label compass_ci_branch=${env.CI_BRANCH} \
        --label compass_ci_build=${env.CI_TAG} \
        ${opts} \
        ${image} \
        bash -c '${cmd}'"""
}

// Prepares Wine in the Windows image container: a fresh prefix in the home folder of the
// container (gone with it). Mono and Gecko are disabled, their install dialogs would block
// wineboot under Xvfb, COMPASS needs neither. Software OpenGL (Mesa) as in the Linux unit
// tests. LANG: Wine maps the Unix file names with the locale charset.
def wineInit() {
    return 'export WINEPREFIX=$HOME/wine WINEDEBUG=-all LANG=C.UTF-8 ' +
           'MESA_GL_VERSION_OVERRIDE=3.3 MESA_GLSL_VERSION_OVERRIDE=330 && ' +
           'WINEDLLOVERRIDES="mscoree,mshtml=" xvfb-run -a sh -c "wineboot -i && wineserver -w"'
}

// Prepares the Wine of the host for the Windows integration tests: a fresh prefix, Mono and
// Gecko disabled (their install dialogs would block wineboot, COMPASS needs neither), no
// crash dialog (it would block the run on the display, a crash ends the process). OpenGL of
// the host GPU. LANG: Wine maps the Unix file names with the locale charset. WINEPATH: the
// Windows PATH of the prefix finds pdflatex.exe.
def wineHostInit(String prefix) {
    return "export PATH='${env.WINE_DIR}/bin':\$PATH WINEPREFIX='${prefix}' WINEPATH='${env.TEXLIVE_WIN_BIN}' WINEDEBUG=-all LANG=C.UTF-8 && " +
           "rm -rf '${prefix}' && mkdir -p '${prefix}' && " +
           "WINEDLLOVERRIDES='mscoree,mshtml=' wineboot -i && " +
           "wine reg add 'HKCU\\Software\\Wine\\WineDbg' /v ShowCrashDialog /t REG_DWORD /d 0 /f && " +
           "wineserver -w && " +
           // the license of the host user, as the Linux tests use it: the client reads it from
           // the home folder of the Wine user (drive_c/users/<user>/.compass/license)
           "mkdir -p '${prefix}/drive_c/users/'\$(id -un)'/.compass' && " +
           "cp -r \$HOME/.compass/license '${prefix}/drive_c/users/'\$(id -un)'/.compass/'"
}

// Runs the integration tests of all checked datasets for one OS build. suiteCmd(dataset,
// suiteArgs) returns the shell command that runs test_suite.py in <RUN_DIR>/scripts/test_infra
// with the given arguments. Each dataset writes into its own output dir
// <TEST_DATA_PATH>/runs/<os>/<dataset>: a shared one makes every run overwrite the previous
// one's results - and junit_results.xml with them, so only the last dataset would reach
// the Jenkins test report.
def runIntegrationTests(String os, Closure suiteCmd) {
    def tagsStr    = selectedTags().join(',')
    def runsDir    = "${env.TEST_DATA_PATH}/runs/${os}"
    def scriptsDir = "${env.RUN_DIR}/scripts"
    def failures   = []

    // the post actions publish the results of the OS builds tested here
    env.TESTED_OS = ((env.TESTED_OS ?: '') + " ${os}").trim()

    sh "rm -rf ${runsDir}"

    for (dataset in selectedDatasets()) {
        echo "Running ${os} tests for dataset: ${dataset.name} (tags: ${tagsStr})${params.ASAN && os == env.LINUX_OS ? ' [ASan]' : ''}"

        def suiteArgs = "--path=\"${scriptsDir}/tests\" --manifest=\"${dataset.manifest}\" " +
                        "--output=\"${runsDir}/${dataset.name}\" --tags=\"${tagsStr}\" " +
                        "--deps=tests --no-prompt --gui " +
                        "--mem-interval=${env.MEM_SAMPLE_INTERVAL_SEC} --mem-warn=${env.MEM_WARN_MB} " +
                        "--cfg-override=none"

        // The exit code of test_suite.py has to survive the `tee`: a shell
        // pipeline reports the status of its LAST command, so without this the
        // suite could crash and the step would still report success. `pipefail`
        // is not used since it is not available in every /bin/sh - instead the
        // status is written to a file inside the pipeline and re-raised after it.
        // Streaming through tee is kept so the Jenkins console shows test
        // progress live. returnStatus keeps the loop going: every dataset runs
        // even if an earlier one failed, the stage is failed afterwards.
        def logBase = "${env.RUN_DIR}/test_${os}_${dataset.name}"
        def rc = sh(returnStatus: true, script: """
            set +e
            cd '${scriptsDir}/test_infra' || exit 2
            { ${suiteCmd(dataset, suiteArgs)} ; \
              echo \$? > '${logBase}.rc' ; } \
                2>&1 | tee '${logBase}.log'
            exit `cat '${logBase}.rc'`
        """)

        // 0 = all tests passed, 1 = tests failed, 2 = suite could not be run
        // (see ExitCode* in test_suite.py)
        if (rc != 0) {
            def reason = (rc == 1) ? 'test failures' : "suite error (exit ${rc})"
            echo "Dataset ${dataset.name} (${os}): ${reason}"
            failures << "${dataset.name} (${reason})"
        }
    }

    // Fail the stage here, after all datasets have run, so the Stage View shows
    // the failure on the integration test stage instead of on the post actions
    // where the junit step publishes results.
    if (failures) {
        error "${os} integration tests failed: ${failures.join(', ')}"
    }
}

pipeline {
    agent any

    options {
        timestamps()
        // Two builds of the same branch share one workspace, so a second build
        // would compile into the build dir of the first one.
        disableConcurrentBuilds()
    }

    parameters {
        string(name: 'EXPERIMENTAL_SRC_BRANCH', defaultValue: '',  description: 'experimental_src branch (empty = same as pipeline branch, fallback to devel)')
        string(name: 'JASTERIX_BRANCH',         defaultValue: 'devel', description: 'jASTERIX branch')

        // Test tags (checkboxes)
        booleanParam(name: 'TAG_SYSTEM',          defaultValue: true, description: 'Tag: system')
        booleanParam(name: 'TAG_IMPORT',          defaultValue: true, description: 'Tag: import')
        booleanParam(name: 'TAG_CALCULATE',       defaultValue: true, description: 'Tag: calculate')
        booleanParam(name: 'TAG_CONTEXT',         defaultValue: true, description: 'Tag: context (Data Context imports and deletes, at_20230422)')
        booleanParam(name: 'TAG_CTX_GUI',         defaultValue: true, description: 'Tags: ctx_gui, ctx_gui_loww (Data Context dialogs, at_20230422 + loww_20260609)')
        booleanParam(name: 'TAG_EVAL',            defaultValue: true, description: 'Tag: eval (+ eval_loww, eval_eddf)')
        booleanParam(name: 'TAG_UI',              defaultValue: true, description: 'Tag: ui (all UI tests)')
        booleanParam(name: 'TAG_VIEWS',           defaultValue: true, description: 'Tag: views')
        booleanParam(name: 'TAG_TABLEVIEW',       defaultValue: true, description: 'Tag: tableview')
        booleanParam(name: 'TAG_HISTOGRAMVIEW',   defaultValue: true, description: 'Tag: histogramview')
        booleanParam(name: 'TAG_SCATTERPLOTVIEW', defaultValue: true, description: 'Tag: scatterplotview')
        booleanParam(name: 'TAG_GEOGRAPHICVIEW',  defaultValue: true, description: 'Tag: geographicview')
        booleanParam(name: 'TAG_ANALYZE',         defaultValue: true, description: 'Tags: analyze, analyze_smr (Analyze Data Source, MLAT + ADS-B, SMR on datasets with CAT010 data)')
        booleanParam(name: 'TAG_ARTAS_SPF',       defaultValue: true, description: 'Tag: artas_spf (ARTAS TRI import/association/display, at_20230422)')
        booleanParam(name: 'TAG_MLAT_RU',         defaultValue: true, description: 'Tag: mlat_ru (MLAT contributing receivers, loww_20260609)')
        booleanParam(name: 'TAG_SENSOR_STATUS',   defaultValue: true, description: 'Tag: sensor_status (CAT063 sensor status, skeyes_20251203)')
        booleanParam(name: 'TAG_TRACKER_CONTRIB', defaultValue: true, description: 'Tag: tracker_contrib (CAT062 contributing sensors, skeyes_20251203)')
        booleanParam(name: 'TAG_LIVE',            defaultValue: true, description: 'Tag: live (live network replay, at_20230422 + skeyes_20251203)')

        // Builds and test runs per OS. With both builds checked, Linux runs first, then Windows.
        booleanParam(name: 'BUILD_LINUX',             defaultValue: true,  description: 'Linux: build, unit tests and AppImage (alma8 image)')
        booleanParam(name: 'LINUX_INTEGRATION_TESTS', defaultValue: true,  description: 'Linux: integration tests against the AppImage (needs BUILD_LINUX)')
        booleanParam(name: 'BUILD_WINDOWS',           defaultValue: false, description: 'Windows: cross build, unit tests in Wine and ZIP file (win_deb12 image)')
        booleanParam(name: 'WIN_INTEGRATION_TESTS',   defaultValue: false, description: 'Windows: integration tests in the Wine of the host, OpenGL on its GPU (needs BUILD_WINDOWS)')

        // Build options
        booleanParam(name: 'CLEAN_BUILD',            defaultValue: false, description: 'Clean build (remove the build dirs before building, Linux and Windows)')
        booleanParam(name: 'ASAN',                   defaultValue: false, description: 'Linux: build with AddressSanitizer (slower; use to diagnose heap/memory corruption)')

        // Datasets (checkboxes)
        booleanParam(name: 'DATASET_05H',    defaultValue: true,  description: 'Dataset: at_20230422_05h (0.5h)')
        // the only one off by default: same data as at_20230422_05h, just a longer slice
        booleanParam(name: 'DATASET_2H',     defaultValue: false, description: 'Dataset: at_20230422_2h (2h)')
        booleanParam(name: 'DATASET_LOWW',   defaultValue: true,  description: 'Dataset: loww_20260609_4h (Vienna airport surface, 4h)')
        booleanParam(name: 'DATASET_SKEYES', defaultValue: true,  description: 'Dataset: skeyes_20251203 (multi-sensor, sensor status + tracker, 7h)')
        booleanParam(name: 'DATASET_EDDF',   defaultValue: true,  description: 'Dataset: eddf_20251117 (Frankfurt airport surface, DGPS drive, 5h)')
    }

    environment {
        GITHUB_TOKEN    = credentials('github-pat')
        CI_DIR          = '/home/user/ci'
        TEST_DATA_PATH  = '/home/user/ci/test'
        // OS_NAME of the build images: names the image, the build dirs and the
        // artifacts (COMPASS_alma8-x86_64.AppImage, build_win_deb12)
        LINUX_OS        = 'alma8'
        LINUX_IMAGE     = "compass/build_${LINUX_OS}"
        WIN_OS          = 'win_deb12'
        WIN_IMAGE       = "compass/build_${WIN_OS}"
        // Wine 10 on the host for the Windows integration tests (built from source, EPEL
        // has no installable Wine for EL 9.8), see readme_ci.md
        WINE_DIR        = '/home/user/opt/wine-10'
        // pdflatex.exe for the PDF report export in Wine: TeX Live on the host with the windows
        // binaries (/home/user/opt/texlive), see readme_ci.md
        TEXLIVE_WIN_BIN = 'Z:\\home\\user\\opt\\texlive\\bin\\windows'
        DISPLAY             = ':0'
        COMPASS_EXTRA_ARGS  = '--no_highdpi -r'
        PYTHONUNBUFFERED    = '1'
        // Container identity, see dockerRun(). Branch names may contain
        // characters that are illegal in a container name.
        CI_BRANCH           = "${env.BRANCH_NAME.replaceAll('[^A-Za-z0-9_.-]', '_')}"
        CI_TAG              = "${env.BRANCH_NAME.replaceAll('[^A-Za-z0-9_.-]', '_')}-${env.BUILD_NUMBER}"
        // Memory sampling of the COMPASS process. Most test cases reuse one
        // instance, so memory accumulates over a dataset run. Build 6 on v100
        // was killed by the OOM killer after 27 silent minutes, with no trace
        // in any test log - the sampler writes results/memory.csv and a warning
        // line into the running test's output before that point is reached.
        MEM_SAMPLE_INTERVAL_SEC = '60'
        // crunch has 61 GB of RAM. The warning has to leave room to act, so it
        // sits well below the point at which the kernel steps in.
        MEM_WARN_MB             = '40000'
    }

    stages {
        stage('Checkout') {
            steps {
                script {
                    // Remove containers left behind by an earlier build of this
                    // branch (aborted build, Jenkins restart). They still hold the
                    // workspace and would compile into it while this build does.
                    sh "docker ps -aq --filter label=compass_ci_branch=${CI_BRANCH} | xargs -r docker rm -f || true"

                    // Run dir of this build on the host: artifacts of all OS builds
                    // (collect_artifacts.sh), test logs, crash logs
                    def stamp = sh(script: 'date +%Y%m%d-%H%M%S', returnStdout: true).trim()
                    env.RUN_DIR = "${CI_DIR}/${stamp}-${BUILD_NUMBER}-${BRANCH_NAME}"
                    echo "run dir: ${env.RUN_DIR}"

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

        // Every OS step runs in osStep(): a failed step skips the remaining ones of its OS
        // (osOk in the when conditions), the other OS continues.
        stage('Linux Build') {
            when { expression { params.BUILD_LINUX } }
            steps {
                script {
                    def cleanFlag = params.CLEAN_BUILD ? '--clean' : ''
                    def asanFlag  = params.ASAN ? '--asan' : ''
                    osStep(LINUX_OS) {
                        sh dockerRun('build', LINUX_IMAGE,
                                     '-v $(pwd):/workspace/compass -v $(pwd)/jasterix:/workspace/jasterix -w /workspace/compass/docker',
                                     "set -e; export WORKSPACE_BASE=/workspace; ./build_jasterix.sh ${cleanFlag} ${asanFlag} && ./build_compass.sh ${cleanFlag} ${asanFlag}")
                    }
                }
            }
        }

        stage('Linux Unit Tests') {
            when { expression { params.BUILD_LINUX && osOk(env.LINUX_OS) } }
            steps {
                script {
                    // When ASAN=true, LSan runs at process exit and returns its own
                    // nonzero exitcode (default 23) when it finds leaks - that would
                    // fail the stage even if all assertions passed. exitcode=0 keeps
                    // leak output visible while letting the real test exit code stand.
                    def lsanEnv = params.ASAN ? "LSAN_OPTIONS=exitcode=0 " : ''
                    // Every stage runs in a fresh container, so the jASTERIX install
                    // of the Build stage is gone - compass_tests links libjasterix.so.
                    // It goes to /usr/lib, which is no default loader path on alma8,
                    // only listed in ld.so.conf => ldconfig after the install.
                    // LANG: the container has the POSIX locale, Qt 5.15 then decodes file
                    // names as ASCII (test_utf8_paths). Desktop sessions are UTF-8.
                    osStep(LINUX_OS) {
                        sh dockerRun('unittests', LINUX_IMAGE,
                                     '-v $(pwd):/workspace/compass -v $(pwd)/jasterix:/workspace/jasterix -w /workspace/compass',
                                     "set -e; sudo cmake --install /workspace/jasterix/build_${LINUX_OS} > /dev/null && sudo ldconfig && ${lsanEnv}LANG=C.UTF-8 MESA_GL_VERSION_OVERRIDE=3.3 MESA_GLSL_VERSION_OVERRIDE=330 xvfb-run -a ./build_${LINUX_OS}/bin/compass_tests")
                    }
                }
            }
        }

        stage('Linux AppImage') {
            when { expression { params.BUILD_LINUX && osOk(env.LINUX_OS) } }
            steps {
                script {
                    def asanFlag = params.ASAN ? '--asan' : ''
                    osStep(LINUX_OS) {
                        sh dockerRun('appimage', LINUX_IMAGE,
                                     "-v \$(pwd):/workspace/compass -v \$(pwd)/jasterix:/workspace/jasterix -v ${CI_DIR}:${CI_DIR} -w /workspace/compass",
                                     "set -e; export WORKSPACE_BASE=/workspace; sudo cmake --install /workspace/jasterix/build_${LINUX_OS} && sudo cmake --install /workspace/compass/build_${LINUX_OS} && sudo ldconfig && cd /workspace/compass/docker && ./deploy_compass.sh ${asanFlag}")
                        sh "bash docker/collect_artifacts.sh \$(pwd) ${env.RUN_DIR} ${LINUX_OS}"
                    }
                }
            }
        }

        // The Windows image runs as root by default: its containers run as the Jenkins user
        // (--user), so that the build files belong to it (as the Linux image user, uid 1000).
        stage('Windows Build') {
            when { expression { params.BUILD_WINDOWS } }
            steps {
                script {
                    def cleanFlag = params.CLEAN_BUILD ? '--clean' : ''
                    osStep(WIN_OS) {
                        sh dockerRun('winbuild', WIN_IMAGE,
                                     '--user $(id -u):$(id -g) -v $(pwd):/workspace/compass -v $(pwd)/jasterix:/workspace/jasterix -w /workspace/compass/docker',
                                     "set -e; export WORKSPACE_BASE=/workspace; ./win_build.sh all ${cleanFlag}")
                    }
                }
            }
        }

        stage('Windows Unit Tests') {
            when { expression { params.BUILD_WINDOWS && osOk(env.WIN_OS) } }
            steps {
                script {
                    // compass_tests.exe runs from a deployment with all DLLs and Qt plugins.
                    // It finds the repository through COMPASS_SOURCE_DIR, the Linux path of the
                    // build (/workspace/compass), which Wine resolves on drive Z:.
                    // The shell timeout fails the Windows build on a hang (e.g. a process that does
                    // not exit), a Jenkins timeout would abort the whole build (osStep).
                    // The second run checks that a failed assertion ends the process while osgEarth
                    // jobs exist (exit_hooks.h): any exit code but 0 and the ones of the timeout.
                    def tests = "/workspace/compass/build_${WIN_OS}/compass_deploy/compass_tests.exe"
                    osStep(WIN_OS) {
                        sh dockerRun('winunittests', WIN_IMAGE,
                                     '--user $(id -u):$(id -g) -v $(pwd):/workspace/compass -w /workspace/compass/docker',
                                     "set -e; export WORKSPACE_BASE=/workspace; ./win_deploy.sh --with_tests; ${wineInit()}; " +
                                     "xvfb-run -a timeout -k 30s 20m wine ${tests}; " +
                                     "rc=0; xvfb-run -a timeout -k 10s 120s wine ${tests} \"[osgearthexitabort]\" >/dev/null 2>&1 || rc=\$?; " +
                                     "echo \"assertion exit check: exit code \$rc\"; [ \$rc -ne 0 ] && [ \$rc -ne 124 ] && [ \$rc -ne 137 ]")
                    }
                }
            }
        }

        stage('Windows ZIP') {
            when { expression { params.BUILD_WINDOWS && osOk(env.WIN_OS) } }
            steps {
                script {
                    osStep(WIN_OS) {
                        sh dockerRun('winzip', WIN_IMAGE,
                                     '--user $(id -u):$(id -g) -v $(pwd):/workspace/compass -w /workspace/compass/docker',
                                     'set -e; export WORKSPACE_BASE=/workspace; ./win_deploy.sh --zip')
                        sh "bash docker/collect_artifacts.sh \$(pwd) ${env.RUN_DIR} ${WIN_OS}"
                    }
                }
            }
        }

        // Integration tests run after all builds, Linux first, then Windows.
        stage('Linux Integration Tests') {
            when { expression { params.BUILD_LINUX && params.LINUX_INTEGRATION_TESTS && osOk(env.LINUX_OS) && hasIntegrationSelection() } }
            steps {
                script {
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
                    def asanEnv = params.ASAN ? "ASAN_OPTIONS='abort_on_error=1:log_path=${env.RUN_DIR}/asan' LSAN_OPTIONS='exitcode=0' ASAN_SYMBOLIZER_PATH=/usr/bin/llvm-symbolizer COMPASS_QUIT_TIMEOUT_SEC=180 " : ''
                    def appimage = "${env.RUN_DIR}/COMPASS_${LINUX_OS}-x86_64.AppImage"

                    // natively on the host, for OpenGL on its GPU (DISPLAY)
                    osStep(LINUX_OS) {
                        runIntegrationTests(LINUX_OS) { dataset, suiteArgs ->
                            "${asanEnv}PYTHONPATH='${env.RUN_DIR}/scripts' python3 test_suite.py --binary='${appimage}' ${suiteArgs}"
                        }
                    }
                }
            }
        }

        stage('Windows Integration Tests') {
            when { expression { params.BUILD_WINDOWS && params.WIN_INTEGRATION_TESTS && osOk(env.WIN_OS) && hasIntegrationSelection() } }
            steps {
                script {
                    osStep(WIN_OS) {
                        // unpacked ZIP file of the build (collect_artifacts.sh), started in Wine
                        // by compass_client_wine.sh
                        def deployDir = sh(script: "ls -d ${env.RUN_DIR}/win64/COMPASS_*_win64 | head -1", returnStdout: true).trim()

                        // natively on the host as the Linux tests, for OpenGL on its GPU (DISPLAY):
                        // Wine of the host (WINE_DIR), a fresh Wine prefix per dataset in the run
                        // dir. Linux paths in the arguments work unchanged, Wine resolves them on
                        // drive Z:. --gui: COMPASS windows on the display.
                        runIntegrationTests(WIN_OS) { dataset, suiteArgs ->
                            "${wineHostInit("${env.RUN_DIR}/wine/${dataset.name}")} && " +
                            "COMPASS_WIN_DEPLOY='${deployDir}' PYTHONPATH='${env.RUN_DIR}/scripts' " +
                            "python3 test_suite.py --binary='${env.RUN_DIR}/compass_client_wine.sh' ${suiteArgs}"
                        }
                    }
                }
            }
        }
    }

    post {
        always {
            // Stop any container of this build that outlived its `docker run`
            // client - on abort Jenkins kills the client, not the container, and
            // the container would keep compiling into the workspace.
            sh "docker ps -aq --filter label=compass_ci_build=${CI_TAG} | xargs -r docker rm -f || true"
            // Crash logs are kept on the host in the run dir
            // ${CI_DIR}/*-${BUILD_NUMBER}-${BRANCH_NAME}/compass_crash_*.log
            // so they're available per-build via SSH but don't clutter the
            // Jenkins artifact list. Workspace cleanup of any leftover copies
            // from earlier pipeline versions that did archive them.
            sh "rm -f compass_crash_*.log"
            // Collect the memory curves and the JUnit XML of each tested OS build and
            // dataset into the workspace - only of the OS builds tested in this build,
            // the runs dir of the others holds the results of an earlier build.
            // memory.csv: an OOM-killed run can be read back from the build artifacts
            // alone. JUnit: one file per dataset, a single file would only ever carry
            // the last dataset's tests. Suite and class names get the OS as prefix,
            // the OS builds run the same test ids.
            sh """
                rm -rf memory_csv junit_results && mkdir -p memory_csv junit_results
                for os in ${env.TESTED_OS ?: ''}; do
                    for d in ${TEST_DATA_PATH}/runs/\$os/*/results; do
                        [ -d "\$d" ] || continue
                        ds=\$(basename \$(dirname "\$d"))
                        if [ -f "\$d/memory.csv" ]; then
                            cp "\$d/memory.csv" "memory_csv/memory_\${os}_\$ds.csv"
                        fi
                        if [ -f "\$d/junit_results.xml" ]; then
                            sed -e 's/<testsuite name="/&'"\$os"'./' -e 's/ classname="/&'"\$os"'./' \
                                "\$d/junit_results.xml" > "junit_results/\${os}_\$ds.xml"
                        fi
                    done
                done
            """
            // Archive test logs and memory curves
            archiveArtifacts artifacts: '**/test_*.log, memory_csv/*.csv', allowEmptyArchive: true
            junit testResults: 'junit_results/*.xml', allowEmptyResults: true
        }
        failure {
            echo 'Build or tests failed!'
        }
        success {
            echo 'All stages completed successfully.'
        }
    }
}
