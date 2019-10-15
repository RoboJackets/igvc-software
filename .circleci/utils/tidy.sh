#!/bin/bash

# some settings
tidied_files_path=".circleci/utils/tidied_files.txt"


# For pretty printing
red=$(tput setaf 1)
green=$(tput setaf 2)
bold=$(tput bold)
rs=$(tput sgr0)
error="$red$bold"
success="$green"

echo "${bold}Starting clang-tidy script...${rs}"

# Check for compile_commands.json
COMPILE_COMMANDS_JSON=../../build/compile_commands.json
if [ ! -f "$COMPILE_COMMANDS_JSON" ]; then
  echo "${error}Couldn't find compile_commands.json at catkin_ws/build/compile_commands.json${rs}"
  echo "Pass -DCMAKE_EXPORT_COMPILE_COMMANDS=ON to catkin_make"
  echo "ie. catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
  exit 1
else
  echo "${success}Found compile_commands.json!${rs}"
fi

# Check for clang-tidy
if [ ! "$(command -v clang-tidy)" ]; then
  echo "${error}Couldn't find clang-tidy${rs}"
  echo "Run sudo apt install clang-tidy clang-tools"
  exit 1
else
  echo "${success}Found clang-tidy!${rs}"
fi

# Check if we want to add '-fix-errors' flag to clang-tidy
fix_errors=""
if [[ $# -eq 1 && "$1" == "-fix" ]]; then
    fix_errors="-fix"
    echo "-fix flag is passed, will pass to clang-tidy!"
    shift
fi

# Check if we want to just run this on one file, or on default files
ignore_strings=""
search_paths=""
if [[ $# -gt 0 ]]; then
  search_paths=$*
else
  search_paths=$(awk 'NR==1' $tidied_files_path)
  ignore_strings=$(awk 'NR==2' $tidied_files_path)
fi
ignore_strings="${ignore_strings// /|}"
search_paths="${search_paths// /|}"

regex='((?!.*('$ignore_strings')))('$search_paths')'

# screw circle ci
num_cores=$(nproc)
if [[ num_cores -eq 32 ]]; then
  num_cores=2
fi

# Run run-clang-tidy
echo "${bold}Running run-clang-tidy.py...${rs}"
echo $regex
set -o xtrace
./.circleci/utils/run-clang-tidy.py -quiet -j $num_cores $fix_errors -p ../../build/ $regex
echo "${bold}Done!${rs}"
