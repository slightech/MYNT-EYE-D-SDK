pipeline {
  agent {
    // docker { image 'ros:kinetic-ros-base-xenial' }
    docker { image 'joinaero/kinetic-ros-opencv-xenial' }
  }

  stages {
    stage('Prepare') {
      steps {
        echo "WORKSPACE: ${env.WORKSPACE}"
        echo 'apt-get ..'
        sh 'apt-get update'
      }
    }
    stage('Init') {
      steps {
        echo 'make init ..'
        sh 'make init INIT_OPTIONS=-y'
      }
    }
    stage('Build') {
      steps {
        echo 'make build ..'
        sh '. /opt/ros/kinetic/setup.sh; make build'
      }
    }
    stage('Install') {
      steps {
        echo 'make install ..'
        sh '. /opt/ros/kinetic/setup.sh; make install'
      }
    }
    /*
    stage('Test') {
      steps {
        echo 'make test ..'
        sh '. /opt/ros/kinetic/setup.sh; make test'
      }
    }
    */
    stage('Samples') {
      steps {
        echo 'make samples ..'
        sh '''
        apt-get install -y libasound2-dev
        . /opt/ros/kinetic/setup.sh; make samples
        '''
      }
    }
    stage('Tools') {
      steps {
        echo 'make tools ..'
        sh '. /opt/ros/kinetic/setup.sh; make tools'
      }
    }
    stage('ROS') {
      steps {
        echo 'make ros ..'
        sh '''
        . /opt/ros/kinetic/setup.sh
        rosdep install --from-paths wrappers/ros/src --ignore-src --rosdistro kinetic -y
        make ros
        '''
      }
    }
    /*
    stage('Clean') {
      steps {
        echo 'clean ..'
        sh '''
        rm -rf /var/lib/apt/lists/*
        '''
      }
    }
    */
  }

  post {
    always {
      echo 'This will always run'
    }
    success {
      echo 'This will run only if successful'
    }
    failure {
      echo 'This will run only if failed'
      mail to: 'mynteye-ci@slightech.com',
      subject: "Failed Pipeline: ${currentBuild.fullDisplayName}",
      body: "Something is wrong with ${env.BUILD_URL}"
    }
    unstable {
      echo 'This will run only if the run was marked as unstable'
    }
    changed {
      echo 'This will run only if the state of the Pipeline has changed'
      echo 'For example, if the Pipeline was previously failing but is now successful'
    }
  }
}
