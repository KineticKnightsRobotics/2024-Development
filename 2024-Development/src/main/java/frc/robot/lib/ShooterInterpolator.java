package frc.robot.lib;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterInterpolator {


    public double data[][] = {
    };

    InterpolatingDoubleTreeMap m_InterpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();


    //PolynomialRegression m_tiltPolynomialRegression

    public ShooterInterpolator() {
        /*
        m_InterpolatingDoubleTreeMap.put(1.305,0.0);
        m_InterpolatingDoubleTreeMap.put(1.47,10.0);
        m_InterpolatingDoubleTreeMap.put(2.085,20.0);
        m_InterpolatingDoubleTreeMap.put(2.65,30.0);
        m_InterpolatingDoubleTreeMap.put(3.405, 30.5);

        /*
        for (int x = 0; x < data.length; x++) {
            m_InterpolatingDoubleTreeMap.put(data[x][0],data[x][1]);
        }
        */
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
        // coefficient * distance^degree 
        (1*(Math.pow(distance,4))) +
        (1*(Math.pow(distance,3))) +
        (1*(Math.pow(distance,2))) + 
        (1*(Math.pow(distance,1)));
        return angle;
    }

}
