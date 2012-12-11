#!/bin/bash

for i in `seq 1 255`; do ./main /dev/ttyUSB0 $i; sleep 0.01; done;
