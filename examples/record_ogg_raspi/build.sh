#!/bin/bash
g++ -Ofast -fno-strict-aliasing main.cpp Adafruit_VS1053.cpp -lwiringPi -lpthread -o main -D VS1053_USE_WIRINGPI