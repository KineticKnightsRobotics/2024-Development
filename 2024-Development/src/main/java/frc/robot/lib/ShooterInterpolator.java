package frc.robot.lib;

public class ShooterInterpolator {

  /*   public double getTilterAimAngle(double distance) {
        if (distance < 1.35 || 3.28 < distance) {
            return 0.0;
        }
        else {
            return 
            +1.9043   *(Math.pow(distance,3))
            -19.4112  *(Math.pow(distance,2))
            +69.6266  *distance
            -55.5631
            ;
        }
*/
          public double getTilterAimAngle(double distance) {
        if (distance < 1.35 || 3.95 < distance) {
            return 0.0;
        }
        else {
            return 





            -12.1146   *(Math.pow(distance,3))
            +95.2764  *(Math.pow(distance,2))
            -235.1371 *distance
            +206.085
            ;
        }

        //polynomial regression...? ...sigma...

        //Using first and second set of data
        /*


                    +8.3197   *(Math.pow(distance,3))
            -60.5249  *(Math.pow(distance,2))
            +157.0143 *distance
            -119.3402
            ;

            +1.6016   *(Math.pow(distance,3))
            -17.9932  *(Math.pow(distance,2))
            +69.2734  *distance
            -54.0351


         * y=1.9043x^3−19.4112x^2+69.6255x−55.5631
         * +1.9043    *Math.pow(distance,3)
         * -19.4112   *Math.pow(distance,2)
         * +69,6266   *distance
         * -55.5631
         */
        
        /* Only first set of data
        -6.7104*(Math.pow(distance, 2))
        +43.5977*distance
        -40.5278;
        */

        /* Equation pre-North Bay
        -9.5824*(Math.pow(distance,3))
        +55.2489*(Math.pow(distance,2))
        -79.7603*distance
        +45.3198;
        */
    }

}
