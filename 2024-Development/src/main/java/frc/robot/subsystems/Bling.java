package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class Bling extends SubsystemBase {


    Spark ledStrip;

    double green  = 0.77;
    double orange = 0.65;
    double red    = 0.61;
    double yellow = 0.69;
    double strobeColour1 = 0.15;
    double strobeColour2 = 0.35;
    double purple = 0.91;
    double blue = 0.87;
    double strobeBlue = -0.09;



    double colourState;

    //yellow0.69


    public Bling() {
        ledStrip = new Spark(9);
    
        colourState = green;
        ledStrip.set(colourState);
    }

    @Override
    public void periodic() {
        if (DriverStation.isTeleop() && Timer.getMatchTime() < 22.5) {
            colourState = blue;
            ledStrip.set(colourState);
        }else{
            ledStrip.set(0.77);
        }



    }


    public Command blinkNote() {
        return Commands.run(
            () -> {
                if (DriverStation.isEnabled() && Timer.getMatchTime() < 22.5) {
                    ledStrip.set(strobeBlue);
                }
                else {
                    ledStrip.set(strobeColour1);
                }
        },
        this
        ).finallyDo(
            () -> {
                if (DriverStation.isEnabled() && Timer.getMatchTime() < 20) {
                    ledStrip.set(blue);
                }
                else {              
                    ledStrip.set(green);
                }
            }
        );
    }  
}
