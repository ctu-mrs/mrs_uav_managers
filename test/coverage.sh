#!/bin/bash

cd ~/hw_api/test_workspace/build

lcov --capture --directory . --output-file coverage.info
lcov --remove coverage.info "*/test/" --output-file coverage.info.removed
lcov --extract coverage.info.removed "*/*workspace/*" --output-file coverage.info.cleaned
genhtml -o coverage_html coverage.info.cleaned | tee /tmp/genhtml.log

COVERAGE_PCT=`cat /tmp/genhtml.log | tail -n 1 | awk '{print $2}'`

echo "Coverage: $COVERAGE_PCT"
