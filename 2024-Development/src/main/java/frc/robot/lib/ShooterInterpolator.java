package frc.robot.lib;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterInterpolator {


    public double data[][] = {
    };

    InterpolatingDoubleTreeMap m_InterpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();

    public ShooterInterpolator() {

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
    }

    public double interpolateAngle(double distance) {
        SmartDashboard.putNumber("Distance to Speaker", distance);
        SmartDashboard.putNumber("Interpolated Angle", m_InterpolatingDoubleTreeMap.get(distance));
        return m_InterpolatingDoubleTreeMap.get(distance);
    }


}
