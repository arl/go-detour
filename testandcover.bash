#!/bin/bash

# This script tests multiple packages and creates a consolidated cover profile
# See https://gist.github.com/hailiang/0f22736320abe6be71ce for inspiration.
# - The list of packages to test is to be specified in $PACKAGES
# - Files to exclude from coverage (helpers, generated code, etc.) are the
#   ones matching the regex in $COVEREXCLUDES.

function die() {
  echo $*
  exit 1
}

# create temporary coverage profile
TMPPROF=$(mktemp -p . -t tmp.profile.XXX.cov)
# remove temp file when we catch a script exit
trap 'rm -rf $TMPPROF' EXIT

# list packages to be tested and covered, one by line.
readonly PACKAGES='
github.com/aurelien-rainone/go-detour/detour
github.com/aurelien-rainone/go-detour/recast
'

# exclude files from coverage report/count (regex)
readonly COVEREXCLUDES='dbg_helper.go|test_helper.go|*_string.go'

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
IFS='
'
for pkg in $PACKAGES
do
    go test -v -covermode=count -coverprofile=$TMPPROF $pkg || ERROR="Error testing $pkg"
    # filter out the excluded files
    tail -n +2 $TMPPROF | egrep -v "${COVEREXCLUDES}" >> profile.cov || die "Unable to append coverage for $pkg"
done

if [ ! -z "$ERROR" ]
then
    die "Encountered error, last error was: $ERROR"
fi
