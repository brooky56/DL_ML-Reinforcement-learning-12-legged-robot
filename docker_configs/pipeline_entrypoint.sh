#!/bin/sh

# ends script execution if any subcommand fails with non-zero code
catkin clean -b --yes

if [ ! -z "${DO_NOT_USE_PYTHON_THREE}" ]; then
	export ROS_PYTHON_VERSION = 3
fi

echo "\e[95mProject tree:\e[0m"
cd ~/catkin_ws/src
tree .

if [ ! -z "${CONTINUE_ON_STAGE_FAIL}" ]; then
	echo "\e[91mCONTINUE_ON_STAGE_FAIL variable is true, pipeline will continue on stage fail\e[0m"
fi

# ----------------------------------

echo "\e[95mRunning lint-cpp stage...\e[0m"

/bin/bash -c "cpplint --filter=-whitespace,-legal/copyright,-readability/multiline_comment,-build/c++11,-runtime/references --recursive */*"

if [ $? -ne 0 ] && [ -z "${CONTINUE_ON_STAGE_FAIL}" ]; then
	echo "\e[91mlint-cpp stage failed with non-zero code. Ending pipeline...\e[0m"
	exit 1
fi

echo "\e[92mlint-cpp stage finished successfully\e[0m"

# ----------------------------------

echo "\e[95mRunning lint-python stage...\e[0m"
cd ~/catkin_ws/src

/bin/bash -c "flake8 --ignore=D400,D415,W503 --per-file-ignore='**/test/**:D' --exclude='**/__init__.py **/setup.py' --docstring-convention=google ."

if [ $? -ne 0 ] && [ -z "${CONTINUE_ON_STAGE_FAIL}" ]; then
	echo "\e[91mlint-python stage failed with non-zero code. Ending pipeline...\e[0m"
	exit 1
fi

echo "\e[92mlint-python stage finished successfully\e[0m"

# ----------------------------------

echo "\e[95mRunning lint-xml stage...\e[0m"
cd ~/catkin_ws/src

xmllint --noout $(find . -type f -name "*.launch" -exec echo {} \;)

if [ $? -ne 0 ] && [ -z "${CONTINUE_ON_STAGE_FAIL}" ]; then
	echo "\e[91mlint-xml stage failed with non-zero code. Ending pipeline...\e[0m"
	exit 1
fi

echo "\e[92mlint-xml stage finished successfully\e[0m"

# ----------------------------------

echo "\e[95mRunning build stage...\e[0m"
cd ~/catkin_ws/src

/bin/bash -c "catkin build"

if [ $? -ne 0 ] && [ -z "${CONTINUE_ON_STAGE_FAIL}" ]; then
	echo "\e[91mbuild stage failed with non-zero code. Ending pipeline...\e[0m"
	exit 1
fi

echo "\e[92mbuild stage finished successfully\e[0m"

# ----------------------------------

echo "\e[95mRunning unit-and-integration-tests stage...\e[0m"
cd ~/catkin_ws/src

/bin/bash -c "cd ~/catkin_ws
			  catkin run_tests
			  source ~/catkin_ws/devel/setup.bash
			  catkin_test_results"

if [ $? -ne 0 ] && [ -z "${CONTINUE_ON_STAGE_FAIL}" ]; then
  echo "\e[91munit-and-integration-tests stage failed with non-zero code. Ending pipeline...\e[0m"
  exit 1
fi

echo "\e[92munit-and-integration-tests stage finished successfully\e[0m"

# ----------------------------------
echo "\e[95mShowing coverage report\e[0m"
#cd ~/catkin_ws/src

# next time I'll do it in perl

# ls -d */ shows the list of directories
# sed -e "s/\///" removes slashes
# sed 's/^/--cover-package /' adds --cover-package flag
# tr '\n' ' ' removes EOL and makes the output in one line
/bin/bash -c "source ~/catkin_ws/devel/setup.bash
			  nosetests --with-coverage $(ls -d */ | sed -e "s/\///" | sed 's/^/--cover-package /' | tr '\n' ' ')"
