#!/bin/bash

# Csys:
#	X: streamwise
#	Y: vertical (lift)
#	Z: spanwise (side force)
# ================= USER INPUT =================

U_MAG=10                       # Freestream velocity magnitude

AOA_LIST=(-10 -5 0 5 10 15 20)      # Angle of attack (deg)
BETA_LIST=(0 5 10 15)	# Sideslip angle (deg)

SOLVER=simpleFoam              # Change if needed

# ==============================================

TEMPLATE="caseTemplate"

deg2rad="3.141592653589793/180.0"

for AOA in "${AOA_LIST[@]}"; do
  for BETA in "${BETA_LIST[@]}"; do

    CASE="U${U_MAG}_a${AOA}_b${BETA}"
    if [ ! -d $CASE ]; then
        echo "Could not find $CASE"
	else
        rm -r $CASE/0 $CASE/40 $CASE/80 $CASE/120 $CASE/160 $CASE/constant \
            $CASE/postProcessing/forceCoeffs1/0 \
            $CASE/postProcessing/forceCoeffs1/40 \
            $CASE/postProcessing/forceCoeffs1/80 \
            $CASE/postProcessing/forceCoeffs1/120
	fi

  done
done

