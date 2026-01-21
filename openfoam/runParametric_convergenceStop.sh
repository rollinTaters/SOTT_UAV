#!/bin/bash

# Csys:
#	X: streamwise
#	Y: vertical (lift)
#	Z: spanwise (side force)
# ================= USER INPUT =================

U_MAG=10                        # Freestream velocity magnitude

AOA_LIST=(-10 -5 0 5 10 15 20)  # Angle of attack (deg)
BETA_LIST=(0 5 10 15)			# Sideslip angle (deg)

SOLVER=simpleFoam        		# Change if needed
CHUNK=40
MAX_ITERS=200
MIN_ITERS=120

TOL_CD=1e-4
TOL_CL=1e-4

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

	ITER=$(foamListTimes -latestTime)

	while [ $((ITER)) -lt $MAX_ITERS ]; do
		# Skip early iterations
		if [ $((ITER)) -ge $MIN_ITERS ]; then
			FC_FILE=$(ls postProcessing/forceCoeffs1/*/coefficient.dat)

			# Standard deviation over last 100 samples
			dCd=$(tail -n 100 $FC_FILE | awk '{print $2}' \
				  | awk '{sum+=$1; sumsq+=$1*$1} END{print sqrt(sumsq/NR - (sum/NR)^2)}')

			dCl=$(tail -n 100 $FC_FILE | awk '{print $5}' \
				  | awk '{sum+=$1; sumsq+=$1*$1} END{print sqrt(sumsq/NR - (sum/NR)^2)}')

			#echo "fc_file: $(tail -n 100 $FC_FILE)"
			echo "dCd/TOL_CD: $dCd/$TOL_CD    dCl/TOL_CL: $dCl/$TOL_CL"
			
			if awk "BEGIN {exit int( ($dCd < $TOL_CD && $dCl < $TOL_CL) ? 0 : 1)}"; then
				echo "Converged at iteration $ITER"
				break
			fi
		fi
		
		echo "Running $SOLVER iterations $ITER → $((ITER+CHUNK))"
		endTime=$((ITER+CHUNK))

		foamDictionary system/controlDict -entry startFrom -set latestTime
		foamDictionary system/controlDict -entry endTime -set $((endTime))

		simpleFoam > log.simpleFoam
  
		ITER=$(foamListTimes -latestTime)

	done

    cd ..
  done
done

