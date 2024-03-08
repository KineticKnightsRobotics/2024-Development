package frc.robot.subsystems;

//rev
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

//wpi
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//robot
import frc.robot.lib.Constants.ConveyerSubsystemConstants;

public class Conveyer extends SubsystemBase {
    DigitalInput lineBreakSensor;
    CANSparkMax conveyerMotorLeft;
    CANSparkMax conveyerMotorRight;
    boolean toggle =false;
    Spark ledController;
    public Conveyer() {
        //lineBreakSensor = new DigitalInput(ConveyerSubsystemConstants.ID_SENSOR_LINEBREAK);
        conveyerMotorLeft = new CANSparkMax(ConveyerSubsystemConstants.ID_MOTOR_CONVEYER_LEFT, CANSparkLowLevel.MotorType.kBrushless);
        conveyerMotorRight = new CANSparkMax(ConveyerSubsystemConstants.ID_MOTOR_CONVEYER_RIGHT, CANSparkLowLevel.MotorType.kBrushless);
        conveyerMotorLeft.setIdleMode(IdleMode.kBrake);
        conveyerMotorRight.setIdleMode(IdleMode.kBrake);
        conveyerMotorLeft.setInverted(true);
        conveyerMotorRight.setInverted(false);
        conveyerMotorLeft.setSmartCurrentLimit(25);
        conveyerMotorRight.setSmartCurrentLimit(25);
        lineBreakSensor = new DigitalInput(2);//new AnalogInput(ConveyerSubsystemConstants.ID_SENSOR_LINEBREAK);
    }
    @Override
    public void periodic() {
        SmartDashboard.putData(this);
        SmartDashboard.putBoolean("Note in Conveyer", getLineBreak());
        SmartDashboard.putNumber("convey_Left", conveyerMotorLeft.getOutputCurrent());
        SmartDashboard.putNumber("convey_Right", conveyerMotorRight.getOutputCurrent());
    }
    public boolean getLineBreak() {return !lineBreakSensor.get();}
    /**
     * Runs Conveyer wheels until the note hits the linebreak, then stop the motors.  
     */
    public Command intakeGamePiece() {
        return 
        Commands.run(
            ()-> {
                conveyerMotorLeft.set(0.5);
                conveyerMotorRight.set(0.5);
            }
        ,this)
        .until(() -> getLineBreak())
        .andThen(
            () ->{
                conveyerMotorLeft.set(0.0);
                conveyerMotorRight.set(0.0);
            }
        ,this)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }
    /**
     * Used to manually set the conveyer motor speeds.
     */
    public Command setConveyerSpeed(double percentOutput) {
        return Commands.runOnce( () -> {
            conveyerMotorLeft.set(percentOutput);
            conveyerMotorRight.set(percentOutput);
            }
        ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }


    public void setBlinkinVoltage(double percentOutput) {
        ledController.set(percentOutput);
    }

    /*
    public void setConveyerSpeed(double percentOutput) {
        conveyerMotorRight.set(percentOutput);
        conveyerMotorLeft.set(-percentOutput);
    }

    public Command runConveyer(double percentOutput) {
        return Commands.runOnce(() -> setConveyerSpeed(percentOutput), this);
    }
    
    */

    public Command setLED(double percentOutput) {
        return Commands.runOnce(() -> setBlinkinVoltage(percentOutput));
    }

    public Command runConveyerContinous(){
        return Commands.run((() -> setConveyerSpeed(0.4)));
    }

    public boolean toggle(boolean myToggle){
        toggle = myToggle;
       return toggle;
    }

    public boolean getToggle(){
        return toggle;
    }
}