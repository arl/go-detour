#!/bin/bash

# This script tests multiple packages and creates a consolidated cover profile
# See https://gist.github.com/hailiang/0f22736320abe6be71ce for inspiration.
# - The list of packages to test is specified in $PACKAGES variables.
# - Files to exclude from coverage (test helpers, generated code, etc.) are the
#   ones matching the regex in $COVEREXCLUDES.

function die() {
  echo $*
  exit 1
}

# list packages to be test and covered, one by line.
readonly PACKAGES='github.com/aurelien-rainone/math32'

# exclude files from coverage report/count (regex)
readonly COVEREXCLUDES=''

export GOPATH=`pwd`:$GOPATH

# Initialize profile.cov
echo "mode: count" > profile.cov

# Initialize error tracking
ERROR=""

# Load environment variables with things like authentication info
if [ -f "./envvars.bash" ]
then
    source "./envvars.bash"
fi

# Test each package and append coverage profile info to profile.cov
go test -v -covermode=count -coverprofile=profile.cov $pkg || ERROR="Error testing $pkg"

if [ ! -z "$ERROR" ]
then
    die "Encountered error, last error was: $ERROR"
fi
