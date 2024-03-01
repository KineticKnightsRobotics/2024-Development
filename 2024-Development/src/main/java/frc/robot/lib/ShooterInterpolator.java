package frc.robot.lib;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterInterpolator {


    public double data[][] = {
        {0,60.0}
    };

    InterpolatingDoubleTreeMap m_InterpolatingDoubleTreeMap;

    public ShooterInterpolator() {
        for (int x = 0; x < data.length; x++) {
            m_InterpolatingDoubleTreeMap.put(data[x][0],data[x][1]);
        }
    }

    public double interpolateAngle(double distance) {
        return m_InterpolatingDoubleTreeMap.get(distance);
    }


}
