package frc.robot.lib;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterInterpolator {


    public double data[][] = {
    };

    InterpolatingDoubleTreeMap m_InterpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();


    //PolynomialRegression m_tiltPolynomialRegression

    public ShooterInterpolator() {
        m_InterpolatingDoubleTreeMap.put(1.305,0.0);
        m_InterpolatingDoubleTreeMap.put(1.47,10.0);
        m_InterpolatingDoubleTreeMap.put(2.085,20.0);
        m_InterpolatingDoubleTreeMap.put(2.65,30.0);
        m_InterpolatingDoubleTreeMap.put(3.405, 30.5);

        for (int x = 0; x < data.length; x++) {
            m_InterpolatingDoubleTreeMap.put(data[x][0],data[x][1]);
        }
        //m_tiltPolynomialRegression = new PolynomialRegression(
        //    Array.listOf, 0);
    }

    public double interpolateAngle(double distance) {
        SmartDashboard.putNumber("Distance to Speaker", distance);
        SmartDashboard.putNumber("Interpolated Angle", m_InterpolatingDoubleTreeMap.get(distance));
        return m_InterpolatingDoubleTreeMap.get(distance);
    }

    public double findAngleRegression(double distance) {
        return 0.0;
    }

    public double returnAngleEquation(double distance) {
        return 0.0;
    }

    public double getTilterAimAngle(double distance) {
        double angle = 
        //y=-26.1742x^{6}+399.3922x^{5}-2507.7064x^{4}+8292.5075x^{3}-15232.2054x^{2}+14739.1237x-5842.1178


        //polynomial regression...? ...sigma...

        -(26.1742*(Math.pow(distance,6)))
        +(399.3922*(Math.pow(distance,5))) 
        -(2507.7064*(Math.pow(distance,4))) 
        +(8292.5075*(Math.pow(distance,3)))
        -(15232.2054*(Math.pow(distance,2)))
        +(14739.1237*(Math.pow(distance,1)))
        -(5842.1178);

        /* 
        (1*(Math.pow(distance,4))) +
        (1*(Math.pow(distance,3))) +
        (1*(Math.pow(distance,2))) + 
        (1*(Math.pow(distance,1)));
        */

        if (distance < 3.6) {
            return angle;
        }
        else {
            return 0.0;
        }

    }

}
