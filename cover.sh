#! /usr/bin/env bash

# This script runs all tests in current package and all sub-packages and performs
# coverage analysis and outputs a coverage profile for each package that was
# tested.
# Coverage profiles are then merged altogether and, depending on the environment
# in which this script is executed, either:
# - send profile to coveralls.io (on continuous integration)
# - open web browser displaying annotated source code.

function die() {
  echo -ne "${*}\n"
  exit 1
}

# retrieve tool paths
if [[ $CI == true ]]; then
  GOVENDOR="$HOME/gopath/bin/govendor"
  GOCOVMERGE="$HOME/gopath/bin/gocovmerge"
  GOVERALLS="$HOME/gopath/bin/goveralls"
else
  GOVENDOR=$(which govendor)
  GOCOVMERGE=$(which gocovmerge)
  GOVERALLS=$(which goveralls)
fi

# check tool paths
[ -z $GOVENDOR ] && die "govendor not found, run 'go get github.com/kardianos/govendor'"
[ -z $GOCOVMERGE ] && die "gocovmerge not found, run 'go get github.com/wadey/gocovmerge'"
[ -z $GOVERALLS ] && die "goveralls not found, run 'go get github.com/mattn/goveralls'"

# create list of project packages, excluding vendored (with govendor)
export PKGS=$($GOVENDOR list -no-status +local)

# make comma-separated
export PKGS_DELIM=$(echo "$PKGS" | paste -sd "," -)

# run with full coverage (including other packages) with govendor
go list -f "{{if or (len .TestGoFiles) (len .XTestGoFiles)}}$GOVENDOR test -covermode count -coverprofile {{.Name}}_{{len .Imports}}_{{len .Deps}}.coverprofile -coverpkg '$PKGS_DELIM' {{.ImportPath}}{{end}}" $PKGS | xargs -I {} bash -c {}

# merge the package specific coverage profiles into one
$GOCOVMERGE `ls *.coverprofile` > cover.out

if [[ $CI == true ]]; then
  # on continuous integration, send to coveralls.io
  $GOVERALLS -coverprofile=cover.out -service=travis-ci
else
  # otherwise, show profile in the browser
  go tool cover -html=cover.out
fi

# cleanup
rm -rf ./cover.out
rm -rf ./*.coverprofile
