#!/bin/bash
./picoemu littleos_pico2_riscv.uf2 -debug -timeout 2 2>&1 | awk '/PC=0x10002C02/ { p=1; c=10 } p>0 { print; c--; if(c==0) p=0 }'
