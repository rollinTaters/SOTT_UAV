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
		echo "Running case: $CASE"

		cp -r "$TEMPLATE" "$CASE"
		cd "$CASE" || exit 1

		# Compute velocity components
		UX=$(echo "$U_MAG * c($AOA*$deg2rad) * c($BETA*$deg2rad)" | bc -l)
		UY=$(echo "$U_MAG * s($AOA*$deg2rad)" | bc -l)
		UZ=$(echo "$U_MAG * c($AOA*$deg2rad) * s($BETA*$deg2rad)" | bc -l)

		echo "U = ($UX $UY $UZ)"

		# Replace placeholders in 0/U
		sed -i "s/UX/${UX}/g" 0/include/initialConditions
		sed -i "s/UY/${UZ}/g" 0/include/initialConditions
		sed -i "s/UZ/${UY}/g" 0/include/initialConditions
	else
		echo "Re-running case: $CASE"
		cp -r "$TEMPLATE/system" "$CASE"
		cd "$CASE" || exit 1
	fi
	
    # Run mesh & solver
    #blockMesh > log.blockMesh
    $SOLVER > log.$SOLVER

    cd ..
  done
done

