pipeline {
    agent any

    parameters {
        booleanParam(name: 'RUN_DB_TESTS',     defaultValue: true,  description: 'Run database test module')
        booleanParam(name: 'RUN_UI_TESTS',     defaultValue: true,  description: 'Run UI test module')
        booleanParam(name: 'RUN_EVAL_TESTS',   defaultValue: true,  description: 'Run evaluation test module')
        booleanParam(name: 'RUN_SYSTEM_TESTS', defaultValue: true,  description: 'Run system test module')
        booleanParam(name: 'RUN_EXTRA_TESTS',  defaultValue: false, description: 'Run extra test module')
    }

    environment {
        GITHUB_TOKEN    = credentials('github-pat')
        CI_DIR          = '/home/user/ci'
        TEST_DATA_PATH  = '/home/user/ci/test'
        DOCKER_IMAGE    = 'compass/build_deb10'
        DISPLAY             = ':0'
        COMPASS_EXTRA_ARGS  = '--no_highdpi -r'
    }

    stages {
        stage('Checkout') {
            steps {
                // Fresh clone experimental_src
                sh 'rm -rf experimental_src'
                sh "git clone --depth 1 --branch ${BRANCH_NAME} https://${GITHUB_TOKEN}@github.com/hpuhr/experimental_src.git experimental_src || git clone --depth 1 --branch devel https://${GITHUB_TOKEN}@github.com/hpuhr/experimental_src.git experimental_src"
                // Fresh clone jASTERIX
                sh 'rm -rf ../jasterix && git clone --depth 1 --branch devel https://github.com/hpuhr/jASTERIX.git ../jasterix'
            }
        }

        stage('Build') {
            steps {
                sh """
                    docker run --rm \
                        -v \$(pwd):/workspace/compass \
                        -v \$(dirname \$(pwd))/jasterix:/workspace/jasterix \
                        -w /workspace/compass/docker \
                        ${DOCKER_IMAGE} \
                        bash -c 'set -e; export WORKSPACE_BASE=/workspace; ./build_jasterix.sh && ./build_compass.sh'
                """
            }
        }

        stage('Unit Tests') {
            steps {
                sh """
                    docker run --rm \
                        -v \$(pwd):/workspace/compass \
                        -v \$(dirname \$(pwd))/jasterix:/workspace/jasterix \
                        -w /workspace/compass \
                        ${DOCKER_IMAGE} \
                        ./build_deb10/bin/compass_tests
                """
            }
        }

        stage('AppImage') {
            steps {
                sh """
                    docker run --rm \
                        -v \$(pwd):/workspace/compass \
                        -v \$(dirname \$(pwd))/jasterix:/workspace/jasterix \
                        -v ${CI_DIR}:${CI_DIR} \
                        -w /workspace/compass \
                        ${DOCKER_IMAGE} \
                        bash -c 'set -e; export WORKSPACE_BASE=/workspace; sudo make -C /workspace/jasterix/build_deb10 install && sudo make -C /workspace/compass/build_deb10 install && cd /workspace/compass/docker && ./deploy_compass.sh'
                """
                // Collect artifacts
                sh "bash docker/collect_artifacts.sh \$(pwd) ${BUILD_NUMBER} ${BRANCH_NAME}"
            }
        }

        stage('Integration Tests') {
            when {
                expression {
                    return params.RUN_DB_TESTS || params.RUN_UI_TESTS || params.RUN_EVAL_TESTS || params.RUN_SYSTEM_TESTS || params.RUN_EXTRA_TESTS
                }
            }
            steps {
                script {
                    // Build modules string from checkboxes
                    def modules = []
                    if (params.RUN_DB_TESTS)     modules << 'db'
                    if (params.RUN_UI_TESTS)     modules << 'ui'
                    if (params.RUN_EVAL_TESTS)   modules << 'eval'
                    if (params.RUN_SYSTEM_TESTS) modules << 'system'
                    if (params.RUN_EXTRA_TESTS)  modules << 'extra'
                    env.TEST_MODULES = modules.join(',')

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

                    // Find all manifests and run tests per dataset
                    def manifests = sh(
                        script: "find ${TEST_DATA_PATH} -maxdepth 2 -name manifest.json -type f",
                        returnStdout: true
                    ).trim().split('\n').findAll { it }

                    for (manifest in manifests) {
                        def datasetDir = sh(script: "dirname '${manifest}'", returnStdout: true).trim()
                        def datasetName = sh(script: "basename '${datasetDir}'", returnStdout: true).trim()

                        echo "Running tests for dataset: ${datasetName} (modules: ${env.TEST_MODULES})"

                        sh """
                            cd '${scriptsDir}/test' && \
                            PYTHONPATH='${scriptsDir}' python3 test_suite.py \
                                --binary='${appimage}' \
                                --path='${scriptsDir}/tests' \
                                --manifest='${manifest}' \
                                --output='${TEST_DATA_PATH}' \
                                --modules='${env.TEST_MODULES}' \
                                --deps=modules \
                                --no-prompt \
                                --cfg-override=none \
                                2>&1 | tee '${runDir}/test_${datasetName}.log'
                        """
                    }
                }
            }
        }
    }

    post {
        always {
            // Archive logs
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
