
#!/bin/bash

SIM_PATH=/home/sara/Bullet/ShortSim
FILES=$SIM_PATH/InitFiles/*
for f in $FILES
do
  OUTPUT=$(echo $f | rev | cut -d '/' -f 1 |  rev | cut -d '.' -f 1)
  CHAIN_LEN=$(echo $OUTPUT | cut -d '_' -f 2)
  echo $SUBSTRING
  ./../a.out 1 $CHAIN_LEN $f > $SIM_PATH/output/$OUTPUT.dat
done
