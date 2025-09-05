#!/bin/bash
set -e

sudo apt-get install doxygen doxygen-latex graphviz

line=$(cat CMakeLists.txt | grep "project(.*)" -o); tmp=${line:8}; proj_name=${tmp:0:${#tmp}-1};
sed -i "s/^\(PROJECT_NAME\s*=\s*\).*$/\1$proj_name/" Doxyfile
doxygen Doxyfile
