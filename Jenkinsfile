pipeline {
    agent any

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

        // Build options
        booleanParam(name: 'CLEAN_BUILD',            defaultValue: false, description: 'Clean build (remove build_deb10 before building)')

        // Datasets (checkboxes)
        booleanParam(name: 'DATASET_05H', defaultValue: true,  description: 'Dataset: at_20230422_05h (0.5h)')
        booleanParam(name: 'DATASET_2H',  defaultValue: true,  description: 'Dataset: at_20230422_2h (2h)')
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
                    // Fresh clone jASTERIX
                    sh "rm -rf ../jasterix && git clone --depth 1 --branch ${jasterixBranch} https://github.com/hpuhr/jASTERIX.git ../jasterix"
                }
            }
        }

        stage('Build') {
            steps {
                script {
                    def cleanFlag = params.CLEAN_BUILD ? '--clean' : ''
                    sh """
                        docker run --rm \
                            -v \$(pwd):/workspace/compass \
                            -v \$(dirname \$(pwd))/jasterix:/workspace/jasterix \
                            -w /workspace/compass/docker \
                            ${DOCKER_IMAGE} \
                            bash -c 'set -e; export WORKSPACE_BASE=/workspace; ./build_jasterix.sh ${cleanFlag} && ./build_compass.sh ${cleanFlag}'
                    """
                }
            }
        }

        stage('Unit Tests') {
            steps {
                sh """
                    docker run --rm --init \
                        -v \$(pwd):/workspace/compass \
                        -v \$(dirname \$(pwd))/jasterix:/workspace/jasterix \
                        -w /workspace/compass \
                        ${DOCKER_IMAGE} \
                        bash -c 'MESA_GL_VERSION_OVERRIDE=3.3 MESA_GLSL_VERSION_OVERRIDE=330 xvfb-run -a ./build_deb10/bin/compass_tests'
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
                    def anyTag = params.TAG_SYSTEM || params.TAG_IMPORT || params.TAG_CALCULATE || params.TAG_EVAL ||
                                 params.TAG_UI || params.TAG_VIEWS || params.TAG_TABLEVIEW ||
                                 params.TAG_HISTOGRAMVIEW || params.TAG_SCATTERPLOTVIEW || params.TAG_GEOGRAPHICVIEW
                    def anyDataset = params.DATASET_05H || params.DATASET_2H
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
                    if (params.TAG_EVAL)            tags << 'eval'
                    if (params.TAG_UI)              tags << 'ui'
                    if (params.TAG_VIEWS)           tags << 'views'
                    if (params.TAG_TABLEVIEW)       tags << 'tableview'
                    if (params.TAG_HISTOGRAMVIEW)   tags << 'histogramview'
                    if (params.TAG_SCATTERPLOTVIEW) tags << 'scatterplotview'
                    if (params.TAG_GEOGRAPHICVIEW)  tags << 'geographicview'
                    def tagsStr = tags.join(',')

                    // Build dataset list from checkboxes
                    def datasets = []
                    if (params.DATASET_05H) datasets << 'at_20230422_05h'
                    if (params.DATASET_2H)  datasets << 'at_20230422_2h'

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
                    for (dataset in datasets) {
                        def manifest = "${TEST_DATA_PATH}/at_20230422/${dataset}.json"

                        echo "Running tests for dataset: ${dataset} (tags: ${tagsStr})"

                        sh """
                            cd '${scriptsDir}/test_infra' && \
                            PYTHONPATH='${scriptsDir}' python3 test_suite.py \
                                --binary='${appimage}' \
                                --path='${scriptsDir}/tests' \
                                --manifest='${manifest}' \
                                --output='${TEST_DATA_PATH}' \
                                --tags='${tagsStr}' \
                                --deps=tests \
                                --no-prompt \
                                --cfg-override=none \
                                2>&1 | tee '${runDir}/test_${dataset}.log'
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
