package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.lib.Constants.IntakeSubsystemConstants;
import frc.robot.lib.PID_Config.IntakeSubsystem.*;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;

/*
 * TODO: Intake
 * Intake Intake Pivot function
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

    private final CANSparkMax intakePivotMotor;

    private final SparkPIDController intakePivotController;
    
    private final RelativeEncoder intakePivotEncoder;

    private double intakePivotController_Reference;

    private SysIdRoutine m_SysIdRoutine;
    private final MutableMeasure<Voltage> m_SysID_Voltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> m_SysID_Position = mutable(Degrees.of(0));
    private final MutableMeasure<Velocity<Angle>> m_SysID_Velocity = mutable(DegreesPerSecond.of(0));

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

        intakePivotEncoder = intakePivotMotor.getEncoder();
        intakePivotEncoder.setPositionConversionFactor(IntakeSubsystemConstants.INTAKE_PIVOT_ROTATIONS_TO_DEGRESS);

        intakePivotController = intakePivotMotor.getPIDController();
        intakePivotController.setP(IntakePivotControllerPID.Proportional);
        intakePivotController.setI(IntakePivotControllerPID.Integral);
        intakePivotController.setD(IntakePivotControllerPID.Derivitive);
        intakePivotController.setOutputRange(-0.8,0.8);

        SmartDashboard.putBoolean("Intake is stuck!", limitSwitchActuate());


        m_SysIdRoutine = 
            new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> {
                        intakePivotMotor.set(volts.in(Volts));
                    },
                    log -> {log
                        .motor("Intake Pivot")
                        .voltage(m_SysID_Voltage.mut_replace(intakePivotMotor.getAppliedOutput() * intakePivotMotor.getBusVoltage(),Volts))
                        .angularPosition(m_SysID_Position.mut_replace(intakePivotEncoder.getPosition(),Degrees))
                        .angularVelocity(m_SysID_Velocity.mut_replace(intakePivotEncoder.getVelocity(),DegreesPerSecond));
                    },
                    this
                )
            );


    }



    /*
    public void actuateIntake(double position) {
        //intakePivotController_Reference = position;
        intakePivotController.setReference(position, ControlType.kPosition);
    }
    */

    public boolean limitSwitchActuate() {
        return (Math.abs(intakePivotEncoder.getPosition() - intakePivotController_Reference) > 0.1) && (Math.abs(intakePivotEncoder.getVelocity()) <= 0.01);
    }
    
    public double getIntakePosition() {
        return intakePivotEncoder.getPosition();
    }

    public void setRollerSpeed(double percentOutput) {
        rollerMotor.set(percentOutput);
    }

    public void stopactuateIntake() {
        intakePivotController.setReference(intakePivotEncoder.getPosition(), ControlType.kPosition);
        intakePivotMotor.set(0.0);
    }

    /*
    public Command setIntakePosition(double position) {
        return Commands.runOnce(() -> actuateIntake(position));
    }
    */

    @Override
    public void periodic() {
    SmartDashboard.putNumber("Intake Actuator Position", getIntakePosition());
    SmartDashboard.putNumber("roller Current", rollerMotor.getOutputCurrent());
    //zeroIntake(0);
    }

    public void zeroIntake(int position){
        intakePivotEncoder.setPosition(position);
    }
    public Command zeroIntakeCommand() {
        return Commands.runOnce(() -> zeroIntake(0));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.dynamic(direction);
    }


    public Command intakeDown() {
        return Commands
        .run(
            () -> {
                intakePivotController.setReference(IntakeSubsystemConstants.Forward_IntakePivot_Position, ControlType.kPosition);
                intakePivotController_Reference = IntakeSubsystemConstants.Forward_IntakePivot_Position;
            }
        )
        .until(() -> intakePivotEncoder.getPosition() == intakePivotController_Reference)
        .andThen(
            () ->{
                rollerMotor.set(0.8);
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
