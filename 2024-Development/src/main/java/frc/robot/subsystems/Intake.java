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


public class Intake extends SubsystemBase {
    
    private final CANSparkMax rollerMotor;

    private final CANSparkMax intakePivotMotor;

    private final SparkPIDController intakePivotController;
    
    private final RelativeEncoder intakePivotEncoder;

    private double intakePivotController_Reference;

    public Intake(){
        rollerMotor = new CANSparkMax(IntakeSubsystemConstants.ID_MOTOR_ROLLER, MotorType.kBrushless);

        rollerMotor.restoreFactoryDefaults();
        rollerMotor.setOpenLoopRampRate(0.0001);
        rollerMotor.setInverted(true);
        rollerMotor.setIdleMode(IdleMode.kBrake);
        rollerMotor.setSmartCurrentLimit(80);

        intakePivotMotor = new CANSparkMax(IntakeSubsystemConstants.ID_MOTOR_INTAKE_PIVOT, MotorType.kBrushless);
        intakePivotMotor.setClosedLoopRampRate(0);
        intakePivotMotor.setIdleMode(IdleMode.kBrake);
        intakePivotMotor.setSmartCurrentLimit(60);
        intakePivotMotor.setInverted(false);

        intakePivotEncoder = intakePivotMotor.getEncoder();
        intakePivotEncoder.setPositionConversionFactor(IntakeSubsystemConstants.INTAKE_PIVOT_ROTATIONS_TO_DEGRESS);

        intakePivotController = intakePivotMotor.getPIDController();
        intakePivotController.setP(IntakePivotControllerPID.Proportional);
        intakePivotController.setI(IntakePivotControllerPID.Integral);
        intakePivotController.setD(IntakePivotControllerPID.Derivitive);
        intakePivotController.setOutputRange(-0.8,0.8);
    }
    
    
    public double getIntakePosition() {
        return intakePivotEncoder.getPosition();
    }

    public void setRollerSpeed(double percentOutput) {
        rollerMotor.set(percentOutput);
    }

    /*
    public Command setIntakePosition(double position) {
        return Commands.runOnce(() -> actuateIntake(position));
    }
    */

    @Override
    public void periodic() {
    SmartDashboard.putNumber("Intake Actuator Position", getIntakePosition());
    SmartDashboard.putNumber("Intake Roller Current", rollerMotor.getOutputCurrent());
    //zeroIntake(0);
    }

    public void zeroIntake(int position){
        intakePivotEncoder.setPosition(position);
    }
    public Command zeroIntakeCommand() {
        return Commands.runOnce(() -> zeroIntake(0));
    }

    public Command intakeDown() {
        return Commands
        .runOnce(
            () -> {
                intakePivotController.setReference(IntakeSubsystemConstants.Forward_IntakePivot_Position, ControlType.kPosition);
                intakePivotController_Reference = IntakeSubsystemConstants.Forward_IntakePivot_Position;
            }
        )
        //.until(() -> intakePivotEncoder.getPosition() == intakePivotController_Reference)
        .andThen(
            () ->{
                rollerMotor.set(1);

            }
        );
    }
    public Command intakeUp() {
        return Commands
        .runOnce(
            () ->{
                rollerMotor.set(0.0);
            }
        ).andThen(
            ()-> {
                intakePivotController.setReference(IntakeSubsystemConstants.Reverse_IntakePivot_Position, ControlType.kPosition);
                intakePivotController_Reference = IntakeSubsystemConstants.Reverse_IntakePivot_Position;
            }
        )
        .until(
            () -> intakePivotEncoder.getPosition() == intakePivotController_Reference
        )
        ;
    }



}
