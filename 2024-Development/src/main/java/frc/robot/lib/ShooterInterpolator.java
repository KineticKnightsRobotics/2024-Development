package frc.robot.lib;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;

public class ShooterInterpolator {


    public double data[][] = {

        /*
        0.0,0.0
        0.5,15
        1.0,25
        1.5,30.5
        2.0,33.5
         */
        {0.0,0.0},
        {0.5,15.0},
        {1.0,25.0},
        {1.5,30.5},
        {2.0,33.5} 
        /*
        {0.00,0.0},
        {0.25,10.0},//,
        {0.50,20.0},
        {0.75,30.0},
        {1.00,35.0},
        {1.25,40.0},
        {1.50,42.5},
        {1.75,45.0},
        {2.00,47.5},
        {2.25,50.0}
        */
    };

    InterpolatingDoubleTreeMap m_InterpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();

    public ShooterInterpolator() {

        m_InterpolatingDoubleTreeMap.put(0.0,0.0);
        m_InterpolatingDoubleTreeMap.put(0.5,15.0);
        m_InterpolatingDoubleTreeMap.put(1.0,25.0);
        m_InterpolatingDoubleTreeMap.put(1.5,30.5);
        m_InterpolatingDoubleTreeMap.put(2.0,33.5);
        /*
        for (int x = 0; x < data.length; x++) {
            m_InterpolatingDoubleTreeMap.put(data[x][0],data[x][1]);
        }
        */
    }

    public double interpolateAngle(double distance) {
        SmartDashboard.putNumber("Distance to Speaker", distance-Units.inchesToMeters(14)-Units.inchesToMeters(39));
        SmartDashboard.putNumber("Interpolated Angle", m_InterpolatingDoubleTreeMap.get(distance));
        return m_InterpolatingDoubleTreeMap.get(distance);
    }


}
