package frc.robot.subsystems;

//rev
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

//wpi
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//robot
import frc.robot.lib.Constants.ConveyerSubsystemConstants;

public class Conveyer extends SubsystemBase {

    //DigitalInput lineBreakSensor;
    AnalogInput lineBreakSensor;
    CANSparkMax conveyerMotorLeft;
    CANSparkMax conveyerMotorRight;

    Spark ledController;

    //AnalogInput test;

    /* Rev Robotics Blinkin takes a PWM signal from 1000-2000us
    * This is identical to a SparkMax motor. 
    *  -1  corresponds to 1000us
    *  0   corresponds to 1500us
    *  +1  corresponds to 2000us
    */

    public Conveyer() {
        //lineBreakSensor = new DigitalInput(ConveyerSubsystemConstants.ID_SENSOR_LINEBREAK);
        conveyerMotorLeft = new CANSparkMax(ConveyerSubsystemConstants.ID_MOTOR_CONVEYER_LEFT, CANSparkLowLevel.MotorType.kBrushless);
        conveyerMotorRight = new CANSparkMax(ConveyerSubsystemConstants.ID_MOTOR_CONVEYER_RIGHT, CANSparkLowLevel.MotorType.kBrushless);

        conveyerMotorLeft.setIdleMode(IdleMode.kBrake);
        conveyerMotorRight.setIdleMode(IdleMode.kBrake);
        conveyerMotorLeft.setSmartCurrentLimit(25);
        conveyerMotorRight.setSmartCurrentLimit(25);
        //conveyerMotorLeft.setOpenLoopRampRate(2);
        //conveyerMotorRight.setOpenLoopRampRate(2);

        //lineBreakSensor = new DigitalInput(0);

        lineBreakSensor = new AnalogInput(ConveyerSubsystemConstants.ID_SENSOR_LINEBREAK);

        ledController = new Spark(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Note in Conveyer", getLineBreak());

        SmartDashboard.putNumber("Linebreak Voltage", lineBreakSensor.getVoltage());


        SmartDashboard.putNumber("LED Controller", ledController.get());
        if (DriverStation.isDisabled()) {
            setLED(0.53);
        }
        SmartDashboard.putNumber("convey_Left", conveyerMotorLeft.getOutputCurrent());
        SmartDashboard.putNumber("convey_Right", conveyerMotorRight.getOutputCurrent());

    }

    public boolean getLineBreak() {
        //return ! lineBreakSensor.get();
        if (lineBreakSensor.getVoltage() > 2.0) {
            return true;
        }
        else {
            return false;
        }

        
    }

    public void setConveyerSpeed(double percentOutput) {
        conveyerMotorRight.set(percentOutput);
        conveyerMotorLeft.set(-percentOutput);
    }

    public void setBlinkinVoltage(double percentOutput) {
        ledController.set(percentOutput);
    }

    public Command runConveyer(double percentOutput) {
        return Commands.runOnce(() -> setConveyerSpeed(percentOutput), this);
    }

    public Command setLED(double percentOutput) {
        return Commands.runOnce(() -> setBlinkinVoltage(percentOutput));
    }
}
