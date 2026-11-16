#!/bin/bash

# Absolute path to the executable software
echo exe : $1
# Absolute path to the test directory (data source)
echo directory : $2

rm SphereBoxOriginal.txt SphereConeOriginal.txt BodyBoxOriginal.txt BodyConeOriginal.txt MixBoxOriginal.txt MixConeOriginal.txt MasterSolidOriginal.txt

$1 -d $2/Sphere/Box/Original/ > SphereBoxOriginal.txt
echo SphereBoxOriginal finished
$1 -d $2/Sphere/Cone/Original/ > SphereConeOriginal.txt
echo SphereConeOriginal finished
$1 -d $2/Body/Box/Original/ > BodyBoxOriginal.txt
echo BodyBoxOriginal finished
$1 -d $2/Body/Cone/Original/ > BodyConeOriginal.txt
echo BodyConeOriginal finished
$1 -d $2/Mix/Box/Original/ > MixBoxOriginal.txt
echo MixBoxOriginal finished
$1 -d $2/Mix/Cone/Original/ > MixConeOriginal.txt
echo MixConeOriginal finished
$1 -d $2/MasterSolid/Original/ -fusion 0 0 1 0 > MasterSolidOriginal.txt
echo MasterSolidOriginal finished
