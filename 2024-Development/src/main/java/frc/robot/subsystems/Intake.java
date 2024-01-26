package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.lib.Constants.IntakeSubsystemConstants;
import frc.robot.lib.PID_Config.IntakeSubsystem.*;



/*
 * TODO: Intake
 * Intake schwoop function
 * find forward position
 * find backwards position
 * zeroing encoder with voltage limit switch
 * 
 * Commands:
 * 
 * Move intake to forward position
 * Manually run roller, stop on end
 * Automatically run roller until Conveyer subsystem linebreak returns true
 * 
 */


public class Intake extends SubsystemBase {
    
    private final CANSparkMax rollerMotor;

    private final CANSparkMax schwoopMotor;

    private final SparkPIDController schwoopController;
    
    private final RelativeEncoder schwoopEncoder;

    private double schwoopController_Reference;

    public Intake(){
        rollerMotor = new CANSparkMax(9, MotorType.kBrushless);

        rollerMotor.restoreFactoryDefaults();
        rollerMotor.setOpenLoopRampRate(2.0);
        rollerMotor.setInverted(true);

        schwoopMotor = new CANSparkMax(IntakeSubsystemConstants.ID_MOTOR_SCHWOOP, MotorType.kBrushless);
        schwoopMotor.setClosedLoopRampRate(2.0);
        schwoopMotor.setIdleMode(IdleMode.kBrake);

        schwoopEncoder = schwoopMotor.getEncoder();
        schwoopEncoder.setPositionConversionFactor(IntakeSubsystemConstants.SCHWOOP_ROTATIONS_TO_DEGRESS);

        schwoopController = schwoopMotor.getPIDController();
        schwoopController.setP(SchwoopControllerPID.Proportional);
        schwoopController.setI(SchwoopControllerPID.Integral);
        schwoopController.setD(SchwoopControllerPID.Derivitive);

        SmartDashboard.putBoolean("Intake is stuck!", limitSwitchActuate());

    }

    public void actuateIntake(double position) {
        schwoopController_Reference = position;
        schwoopController.setReference(position, ControlType.kPosition);
    }

    public boolean limitSwitchActuate() {
        return (Math.abs(schwoopEncoder.getPosition() - schwoopController_Reference) > 0.1) && (Math.abs(schwoopEncoder.getVelocity()) <= 0.01);
    }
    
    public double getIntakePosition() {
        return schwoopEncoder.getPosition();
    }

    public void setRollerSpeed(double percentOutput) {
        rollerMotor.set(percentOutput);
    }

    public void stopactuateIntake() {
        schwoopController.setReference(schwoopEncoder.getPosition(), ControlType.kPosition);
        schwoopMotor.set(0.0);
    }

    public Command setIntakePosition(double position) {
        return Commands.runOnce(() -> actuateIntake(position));
    }



    public void unlockSchwoop(boolean unlocked) {
        if (unlocked) {schwoopMotor.setIdleMode(IdleMode.kCoast);}
        else          {schwoopMotor.setIdleMode(IdleMode.kBrake);}
    }
}
