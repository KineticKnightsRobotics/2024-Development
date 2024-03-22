package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Bling extends SubsystemBase {


    Spark ledStrip;

    public Bling() {
        ledStrip = new Spark(9);
        ledStrip.set(0.73);
    }






    @Override
    public void periodic() {

    }
    
}
