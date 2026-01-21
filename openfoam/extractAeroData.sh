#!/usr/bin/env bash

echo "U AOA BETA Cd Cl Cs CmRoll CmPitch CmYaw" > aeroResults.dat

for CASE in ./U*; do
	U_MAG=$(echo "$CASE" | sed 's/.*U\([0-9]\+\).*/\1/')
    AOA=$(echo $CASE | sed 's/.*_a\([-0-9]*\)_b.*/\1/')
    BETA=$(echo $CASE | sed 's/.*_b\([-0-9]*\).*/\1/')

    FC_FILE=$(ls $CASE/postProcessing/forceCoeffs1/*/coefficient.dat)
    
    read Cd Cl Cs CmRoll CmPitch CmYaw <<< $(
        awk 'END {print $2, $5, $11, $9, $8, $10}' "$FC_FILE"
    )

    if [ -z "$Cd" ]; then
    	echo "⚠️  No data for $CASE"
    	continue
	fi

    echo "$U_MAG $AOA $BETA $Cd $Cl $Cs $CmRoll $CmPitch $CmYaw" \
        >> aeroResults.dat
done

