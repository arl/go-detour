#! /usr/bin/env bash

GOVENDOR="$HOME/gopath/bin/govendor"
GOCOVMERGE="$HOME/gopath/bin/gocovmerge"
GOVERALLS="$HOME/gopath/bin/goveralls"
#GOVENDOR=govendor
#GOCOVMERGE=gocovmerge
#GOVERALLS=goveralls

# create list of project packages, excluding vendored (with govendor)
export PKGS=$($GOVENDOR list -no-status +local)

# make comma-separated
export PKGS_DELIM=$(echo "$PKGS" | paste -sd "," -)

# run with full coverage (including other packages) with govendor
go list -f "{{if or (len .TestGoFiles) (len .XTestGoFiles)}}$GOVENDOR test -covermode count -coverprofile {{.Name}}_{{len .Imports}}_{{len .Deps}}.coverprofile -coverpkg '$PKGS_DELIM' {{.ImportPath}}{{end}}" $PKGS | xargs -I {} bash -c {}

$GOCOVMERGE `ls *.coverprofile` > cover.out
$GOVERALLS -coverprofile=cover.out -service=travis-ci
rm -rf ./cover.out ./*.coverprofile
