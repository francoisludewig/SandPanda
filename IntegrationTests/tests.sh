#!/bin/bash

# Absolute path to the executable software
echo exe : $1
# Absolute path to the test directory (data source)
echo directory : $2

rm SphereBoxNew.txt SphereConeNew.txt BodyBoxNew.txt BodyConeNew.txt MixBoxNew.txt MixConeNew.txt MasterSolidNew.txt
rm SphereBoxDiff.txt SphereConeDiff.txt BodyBoxDiff.txt BodyConeDiff.txt MixBoxDiff.txt MixConeDiff.txt MasterSolidDiff.txt
rm SphereBoxTiming.txt SphereConeTiming.txt BodyBoxTiming.txt BodyConeTiming.txt MixBoxTiming.txt MixConeTiming.txt MasterSolidTiming.txt

$1 -d $2/Sphere/Box/New/ > SphereBoxNew.txt
diff -ENwbur $2/Sphere/Box/Original/Start_stop/ $2/Sphere/Box/New/Start_stop/ > SphereBoxDiff.txt
diff SphereBoxNew.txt SphereBoxOriginal.txt > SphereBoxTiming.txt
echo SphereBoxNew finished
$1 -d $2/Sphere/Cone/New/ > SphereConeNew.txt
diff -ENwbur $2/Sphere/Cone/Original/Start_stop/ $2/Sphere/Cone/New/Start_stop/ > SphereConeDiff.txt
diff SphereConeNew.txt SphereConeOriginal.txt > SphereConeTiming.txt
echo SphereConeNew finished
$1 -d $2/Body/Box/New/ > BodyBoxNew.txt
diff -ENwbur $2/Body/Box/Original/Start_stop/ $2/Body/Box/New/Start_stop/ > BodyBoxDiff.txt
diff BodyBoxNew.txt BodyBoxOriginal.txt > BodyBoxTiming.txt
echo BodyBoxNew finished
$1 -d $2/Body/Cone/New/ > BodyConeNew.txt
diff -ENwbur $2/Body/Cone/Original/Start_stop/ $2/Body/Cone/New/Start_stop/ > BodyConeDiff.txt
diff BodyConeNew.txt BodyConeOriginal.txt > BodyConeTiming.txt
echo BodyConeNew finished
$1 -d $2/Mix/Box/New/ > MixBoxNew.txt
diff -ENwbur $2/Mix/Box/Original/Start_stop/ $2/Mix/Box/New/Start_stop/ > MixBoxDiff.txt
diff MixBoxNew.txt MixBoxOriginal.txt > MixBoxTiming.txt
echo MixBoxNew finished
$1 -d $2/Mix/Cone/New/ > MixConeNew.txt
diff -ENwbur $2/Mix/Cone/Original/Start_stop/ $2/Mix/Cone/New/Start_stop/ > MixConeDiff.txt
diff MixConeNew.txt MixConeOriginal.txt > MixConeTiming.txt
echo MixConeNew finished

$1 -d $2/MasterSolid/New/ -fusion 0 0 1 0 > MasterSolidNew.txt
diff -ENwbur $2/MasterSolid/Original/Start_stop/ $2/MasterSolid/New/Start_stop/ > MasterSolidDiff.txt
diff MasterSolidNew.txt MasterSolidOriginal.txt > MasterSolidTiming.txt
echo MasterSolidNew finished


cd Analyse/
./MakeReport
