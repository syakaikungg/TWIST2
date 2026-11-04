#!/bin/bash
# Read only: PICO TCP connections and a 10 s sample of the PICO stream through the PC Service.
echo "TCP peers on 63901:"; ss -tn state established '( sport = :63901 )' | tail -n +2
cd ~ && timeout 40 ~/miniforge3/envs/gmr/bin/python ~/nextech/arm_ik/pico_stream_check.py 2>&1 | grep -E 'samples|timestamp'
