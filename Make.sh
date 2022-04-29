#!/bin/sh

if [ ! -d "./Build" ]; then
    echo "create directory build"
    mkdir Build
fi
echo "Build exist"

cd ./Build
# rm -r ./*
# sync
echo "configure project"
cmake .. 
echo "make project"
make -j
echo "Make Done"

