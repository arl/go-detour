#! /usr/bin/env bash

echo "mode: set" > acc.out
for Dir in $(find ./* -maxdepth 10 -type d );
do
  if ls "${Dir}/*.go" &> /dev/null;
  then
    go test -coverprofile=profile.out "${Dir}" -v
    if [ -f profile.out ]
    then
      grep -v "mode: set" profile.out >> acc.out
    fi
fi
done

"${HOME}/gopath/bin/goveralls" -coverprofile=acc.out -service=travis-ci
rm -rf ./profile.out
rm -rf ./acc.out
