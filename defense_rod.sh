#!/bin/sh

echo "Ready...OK."
g++ -Wall -o exe_rod rod.cpp -I ~/Software/local/include/ `ode-config --cflags --libs` -ldrawstuff -framework GLUT -framework OpenGL || { echo oops!; exit 1; }
echo "Done...GO.\n>>> - - - "
./exe_rod