#!/bin/bash

while [ ! -e "build/COLCON_IGNORE" ]; do
  cd ..
  if [[ `pwd` == "/" ]]; then
    # we reached the root and didn't find the build/COLCON_IGNORE file - that's a fail!
    echo "Cannot compile, probably not in a workspace (if you want to create a new workspace, call \"colcon init\" in its root first)".
    exit 1
  fi
done

WORKSPACE_NAME=${PWD##*/}

cd build

lcov --ignore-errors negative --capture --directory . --output-file coverage.info
lcov --remove coverage.info "*/test/*" --output-file coverage.info.removed
lcov --extract coverage.info.removed "*/${WORKSPACE_NAME}/src/*" --output-file coverage.info.cleaned
genhtml --title "MRS UAV System - Test coverage report" --demangle-cpp --legend --frames --show-details -o coverage_html coverage.info.cleaned | tee /tmp/genhtml.log

COVERAGE_PCT=`cat /tmp/genhtml.log | tail -n 1 | awk '{print $2}'`

echo "Coverage: $COVERAGE_PCT"

xdg-open coverage_html/index.html
