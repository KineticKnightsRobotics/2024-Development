package frc.robot.subsystems;

//rev
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

//wpi
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//robot
import frc.robot.lib.Constants.ConveyerSubsystemConstants;

public class Conveyer extends SubsystemBase {

    CANSparkMax conveyerMotor;

    DigitalInput lineBreakSensor;
    
    public Conveyer() {
        conveyerMotor = new CANSparkMax(ConveyerSubsystemConstants.ID_MOTOR_CONVEYER, CANSparkLowLevel.MotorType.kBrushless);

        conveyerMotor.setOpenLoopRampRate(2);


        lineBreakSensor = new DigitalInput(ConveyerSubsystemConstants.ID_SENSOR_LINEBREAK);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Ball in Conveyer", false);
    }

    public boolean getLineBreak() {
        return ! lineBreakSensor.get();
    }

    public void setConveyerSpeed(double percentOutput) {
        conveyerMotor.set(percentOutput);
    }

    public Command runConveyer(double percentOutput) {
        return Commands.runOnce(() -> setConveyerSpeed(percentOutput), this);
    }


}
