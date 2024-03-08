package frc.robot.lib;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterInterpolator {


    public double data[][] = {
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
    };

    InterpolatingDoubleTreeMap m_InterpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();

    public ShooterInterpolator() {
        for (int x = 0; x < data.length; x++) {
            m_InterpolatingDoubleTreeMap.put(data[x][0],data[x][1]);
        }
    }

    public double interpolateAngle(double distance) {
        return m_InterpolatingDoubleTreeMap.get(distance);
    }


}
