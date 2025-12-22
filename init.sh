#!/bin/bash

CMSIS_DIR=cmsis/

git clone --depth 1 --branch 'v6.3.0-rc0' https://github.com/ARM-software/CMSIS_6.git $CMSIS_DIR/cmsis_core
git clone --depth 1 --branch 'v4.3.5' https://github.com/STMicroelectronics/cmsis_device_f1.git $CMSIS_DIR/cmsis_f1
