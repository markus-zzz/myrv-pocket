#!/bin/bash

set -x -e

DIST=dist
STAGING=_staging_
DATE=$(date +'%Y-%m-%d')
VERSION=$1

rm -f MyRV-Pocket.zip
rm -rf ${STAGING}

quartus_sh --flow compile ./src/fpga/ap_core.qpf

cp -r ${DIST} ${STAGING}

python3 utils/reverse-bits.py src/fpga/output_files/ap_core.rbf ${STAGING}/Cores/markus-zzz.MyRV/bitstream.rbf_r

sed -i "s/VERSION/${VERSION}/" ${STAGING}/Cores/markus-zzz.MyRV/core.json
sed -i "s/DATE_RELEASE/${DATE}/" ${STAGING}/Cores/markus-zzz.MyRV/core.json

pushd ${STAGING}
zip -r ../MyRV-Pocket.zip .
popd

cat src/fpga/output_files/ap_core.fit.summary
