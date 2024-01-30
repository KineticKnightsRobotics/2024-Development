package frc.robot.subsystems;

//rev
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

//wpi
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//robot
import frc.robot.lib.Constants.ConveyerSubsystemConstants;

public class Conveyer extends SubsystemBase {

    DigitalInput lineBreakSensor;
    CANSparkMax conveyerMotorLeft;
    CANSparkMax conveyerMotorRight;

    AnalogInput test;

    public Conveyer() {
        //lineBreakSensor = new DigitalInput(ConveyerSubsystemConstants.ID_SENSOR_LINEBREAK);
        conveyerMotorLeft = new CANSparkMax(ConveyerSubsystemConstants.ID_MOTOR_CONVEYER_LEFT, CANSparkLowLevel.MotorType.kBrushless);
        conveyerMotorRight = new CANSparkMax(ConveyerSubsystemConstants.ID_MOTOR_CONVEYER_RIGHT, CANSparkLowLevel.MotorType.kBrushless);
        
        //conveyerMotorLeft.setOpenLoopRampRate(2);
        //conveyerMotorRight.setOpenLoopRampRate(2);

        lineBreakSensor = new DigitalInput(0);

        test = new AnalogInput(1);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Note in Conveyer", getLineBreak());

        SmartDashboard.putNumber("test", test.getVoltage());
    }

    public boolean getLineBreak() {
        return ! lineBreakSensor.get();
    }

    public void setConveyerSpeed(double percentOutput) {
        conveyerMotorRight.set(percentOutput);
        conveyerMotorLeft.set(-percentOutput);
    }

    public Command runConveyer(double percentOutput) {
        return Commands.runOnce(() -> setConveyerSpeed(percentOutput), this);
    }
}
