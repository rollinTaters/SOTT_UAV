


    --== Flight Computer ==--

This piece of code lives on an arduino nano board, because, as always, thats what i had lying around.
And its small enough to shove into the fuselage of a small plane with some other electronic boards as friends.

This piece is responsible for flying the aircraft.
Be it fully autonomous (kinda), when the connection to the ground base inevitably fails.
Or semi automomonous when the pilot wants some hand holding. The system is fly-by-wire and wireless. (so just fly-by- ?)

Flight modes:
- Raw: the pilot directly controls the control surfaces. The way the god almighty intended airplanes to fly.
- Protected: the pilot still has direct control, but FC keeps the plane from dangerous shit. (like stalls and over-g maneuvers)
- Suggestive: the pilot sets the target state, and realizes that the system is considering their suggestion.
- ( Mission computer can send Suggestive mode commands to the FC to embark on fully autonomonueus flights and/or missions )

This piece of code:
- reads BNO055 IMU
- reads Mateksys Airspeed sensor
- writes PWM servo commands
- Communicates with nRF24L0PA to the ground station
- Communicates with Serial to the Mission Computer
