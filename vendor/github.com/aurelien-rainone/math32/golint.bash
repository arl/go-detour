#!/bin/bash

if [ "$TRAVIS" == "true" ]
then
  GOLINT="${HOME}/gopath/bin/golint"
else
  GOLINT=golint
fi
EXCLUDES='vendor|_string.go'

find . -name '*.go' -print | egrep -v "${EXCLUDES}" | xargs $GOLINT -set_exit_status
