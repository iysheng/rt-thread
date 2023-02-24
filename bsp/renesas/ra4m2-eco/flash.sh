#!/bin/sh

pyocd flash --target=R7FA4M2AD --erase=auto --frequency=1000000 rtthread.elf
