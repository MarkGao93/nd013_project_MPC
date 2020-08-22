#!/bin/bash

if [ "mpc" ]; then
	rm mpc
fi

#if [ -d "build" ]; then
#	rm -d -r build
#fi

if [ ! -d "build" ]; then
	mkdir build
fi

cd build

cmake ..
make

cp ./mpc ../mpc

cd ..

./mpc
