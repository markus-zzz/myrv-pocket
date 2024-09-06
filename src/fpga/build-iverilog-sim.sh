#!/bin/bash

set -e
set -x

pushd core/zzz-rv
make clean
make out/soc.v
popd

iverilog sim/sim-top.v sim/mt48lc16m16a2.v core/spram.v core/sprom.v core/core_top.v core/core_bridge_cmd.v apf/common.v core/soc.v -D __SIMULATION__ -o core_top.vvp

