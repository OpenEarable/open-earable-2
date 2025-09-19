#!/bin/bash

xxd -i -n textdata_bin /Users/miguelkuttner/utm/win11/textdata.bin > ./src/drivers/xtensa/textdata.h
xxd -i -n vectors_bin /Users/miguelkuttner/utm/win11/vectors.bin > ./src/drivers/xtensa/vectors.h