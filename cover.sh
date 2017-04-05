#! /usr/bin/env bash
## Perform the coverage analysis of all Go packages, merge them and, depending
## on current environment:
## - send profile to coveralls.io (on continuous integration)
## - open web browser displaying annotated source code.
##
## Usage: cover.sh [--keep]
##
## Options:
##   -h, --help    Display this message.
##   -n            Dry-run; only show what would be done.
##   -k, --keep    Do not remove generated report profiles.
##   --dont-send   Do not send to goveralls nor open the browser
##                 (implies --keep)
##   -x, --exclude Exclude package paths containing a given string.
##

usage() {
  [ "$*" ] && echo "$0: $*"
  sed -n '/^##/,/^$/s/^## \{0,1\}//p' "$0"
  exit 2
} 2>/dev/null

function die() {
  echo -ne "${*}\n"
  exit 1
}

main() {

  # default values
  DRY_RUN=0
  KEEP=0
  DONT_SEND=0
  EXCLUDE=0

  while [ $# -gt 0 ]; do
    case $1 in
    (-n)
      DRY_RUN=1
      shift
      ;;
    (-k|--keep)
      KEEP=1;
      shift
      ;;
    (--dont-send)
      DONT_SEND=1;
      KEEP=1;
      shift
      ;;
    (-x|--exclude)
      EXCLUDE=$2;
      shift
      shift
      ;;
    (-h|--help)
      usage 2>&1
      ;;
    (--)
      shift;
      break;;
    (-*)
      usage "$1: unknown option"
      ;;
    (*)
      break
      ;;
    esac
  done

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
  [ -z "$GOVENDOR" ] && die "govendor not found, run 'go get github.com/kardianos/govendor'"
  [ -z "$GOCOVMERGE" ] && die "gocovmerge not found, run 'go get github.com/wadey/gocovmerge'"
  [ -z "$GOVERALLS" ] && die "goveralls not found, run 'go get github.com/mattn/goveralls'"

  # create list of project packages, excluding vendored (with govendor)
  if [ $EXCLUDE -eq 0 ]; then
    PKGS=$($GOVENDOR list -no-status +local)
  else
    PKGS=$($GOVENDOR list -no-status +local | grep -v $EXCLUDE)
  fi
  export PKGS

  # make comma-separated
  PKGS_DELIM=$(echo "$PKGS" | paste -sd "," -)
  export PKGS_DELIM

  if [ $DRY_RUN -eq 0 ]; then
    # run with full coverage (including other packages) with govendor
    go list -f "{{if or (len .TestGoFiles) (len .XTestGoFiles)}}$GOVENDOR test -covermode count -coverprofile {{.Name}}_{{len .Imports}}_{{len .Deps}}.coverprofile -coverpkg '$PKGS_DELIM' {{.ImportPath}}{{end}}" $PKGS | xargs -I {} bash -c {}
  else
    # dry-run: shows command line
    go list -f "{{if or (len .TestGoFiles) (len .XTestGoFiles)}}$GOVENDOR test -covermode count -coverprofile {{.Name}}_{{len .Imports}}_{{len .Deps}}.coverprofile -coverpkg '$PKGS_DELIM' {{.ImportPath}}{{end}}" $PKGS
    exit
  fi

  # merge the package specific coverage profiles into one
  "$GOCOVMERGE" $(ls *.coverprofile) > cover.out

  if [ $DONT_SEND -eq 0 ]; then
    if [[ $CI == true ]]; then
      # on continuous integration, send to coveralls.io
      $GOVERALLS -coverprofile=cover.out -service=travis-ci
    else
      # otherwise, show profile in the browser
      go tool cover -html=cover.out
    fi
  fi

  if [ $KEEP -eq 0 ]; then
    # cleanup
    rm -rf ./cover.out
    rm -rf ./*.coverprofile
  fi
}

main "$@"
