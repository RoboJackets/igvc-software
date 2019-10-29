# This file is used for aliases for long commands such as clang-format
files := $(shell find . \( -name '*.h' -or -name '*.hpp' -or -name '*.cpp' \) -not -path "./external/*")

format:
	clang-format -i $(files)

autofix: format tidy-fix

tidy:
	.circleci/utils/tidy.sh

tidy-fix:
	.circleci/utils/tidy.sh -fix
