#!/bin/bash
FILE=`readlink -f $0`
DIR=`dirname $FILE`
if pushd $DIR; then
  wget -c -P datasets -i download_benchmark_data.urls
fi
popd > /dev/null
