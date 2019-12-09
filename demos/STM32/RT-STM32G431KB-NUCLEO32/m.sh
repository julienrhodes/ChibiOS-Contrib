#!/bin/bash
set -x
make CHIBIOS_CONTRIB=$(readlink -f ../../../) CHIBIOS=$(readlink -f ../../../../ChibiOS-RT)
