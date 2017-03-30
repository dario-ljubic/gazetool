#!/bin/bash

curl http://dlib.net/files/dlib-19.4.tar.bz2 -o dlib-19.4.tar.bz2
tar -xvjf dlib-19.4.tar.bz2
rm -rf dlib-19.4.tar.bz2
mv dlib-19.4/* .
rm -rf dlib-19.4
