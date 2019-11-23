#!/bin/bash

# some settings
tidied_files_path=".circleci/utils/tidied_files.txt"

# Ignore pb.c / pb.h generated files
IGNORE_REGEX="pb\.[ch]"


# For pretty printing
if test -t 1; then
    # see if it supports colors...
    ncolors=$(tput colors)

    if test -n "$ncolors" && test $ncolors -ge 8; then
      red=$(tput setaf 1)
      green=$(tput setaf 2)
      bold=$(tput bold)
      rs=$(tput sgr0)
      error="$red$bold"
      success="$green"
    fi
fi

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
if [ ! "$(command -v clang-tidy-8)" ]; then
  echo "${error}Couldn't find clang-tidy-8${rs}"
  echo "Run sudo apt install clang-tidy-8 clang-tools-8"
  exit 1
else
  echo "${success}Found clang-tidy-8!${rs}"
fi

# Check if we want to add '-fix-errors' flag to clang-tidy
fix_errors=""
if [[ $# -eq 1 && "$1" == "-fix" ]]; then
    fix_errors="-fix -fix-errors"
    echo "-fix flag is passed, will pass to clang-tidy!"
    shift
fi

# screw circle ci
num_cores=$(nproc)
if [[ num_cores -eq 36 ]]; then
  num_cores=2
fi

# ===================
# | Find diff files |
# ===================

# Find the merge base compared to master.
base=$(git merge-base refs/remotes/origin/master HEAD)
# Create an empty array that will contain all the filepaths of files modified.
modified_filepaths=()

# To properly handle file names with spaces, we have to do some bash magic.
# We set the Internal Field Separator to nothing and read line by line.
while IFS='' read -r line
do
  # For each line of the git output, we call `realpath` to get the absolute path of the file.
  absolute_filepath=$(realpath "$line")

  # Append the absolute filepath.
  if echo "$absolute_filepath" | grep -q -E '(\.cpp)|(\.h)' | grep -v -E "${IGNORE_REGEX}" ; then
    echo "$absolute_filepath"
    modified_filepaths+=("$absolute_filepath")
  fi

# `git diff-tree` outputs all the files that differ between the different commits.
# By specifying `--diff-filter=d`, it doesn't report deleted files.
# Also get list of modified files so you don't need to commit
done < <(git diff-tree --no-commit-id --diff-filter=d --name-only -r "$base" HEAD && git ls-files -m)

# =====================
# | Remove duplicates |
# =====================
modified_filepaths=($(echo "${modified_filepaths[@]}" | tr ' ' '\n' | sort -u | tr '\n' ' '))

if [ ${#modified_filepaths[@]} -eq 0 ]; then
  echo "No changed cpp files!"
  exit 0
fi

# =========================
# | Run with GNU parallel |
# =========================

# -m specifies that `parallel` should distribute the arguments evenly across the executing jobs.
# -p Tells clang-tidy where to find the `compile_commands.json`.
# `{}` specifies where `parallel` adds the command-line arguments.
# `:::` separates the command `parallel` should execute from the arguments it should pass to the commands.
# `| tee` specifies that we would like the output of clang-tidy to go to `stdout` and also to capture it in
# `$build_dir/clang-tidy-output` for later processing.
build_dir="../../build/"
parallel -m clang-tidy-8 -p $build_dir ${fix_errors} {} ::: "${modified_filepaths[@]}" | tee "$build_dir/clang-tidy-output"

# ===============================
# | Convert result to JUnit XML |
# ===============================

output_dir=$build_dir/test-results/clang-tidy
mkdir -p $output_dir
cat "$build_dir/clang-tidy-output" | ./.circleci/utils/clang-tidy-to-junit.py "$(pwd)" >"$output_dir/results.xml"

if [ -s "$build_dir/clang-tidy-output" ]; then
  echo "clang-tidy detected errors!"
  exit 1
fi

exit 0
