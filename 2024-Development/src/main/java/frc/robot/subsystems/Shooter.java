package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
//import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.PID_Config.ShooterSubsystem.ShooterVelocityPID;
import frc.robot.lib.PID_Config.ShooterSubsystem.TilterPIDConfig;
//import frc.robot.lib.PID_Config.ShooterSubsystem.TilterPIDConfig;
import frc.robot.lib.Constants.ShooterSubsystemConstants;
import frc.robot.lib.PID_Config;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;


public class Shooter extends SubsystemBase {

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));
    //
    private final MutableMeasure<Angle> m_degrees = mutable(Degrees.of(0));
    private final MutableMeasure<Velocity<Angle>> m_velocity_degrees = mutable(DegreesPerSecond.of(0));

    private final SimpleMotorFeedforward SHOOTER_FEEDFORWARD_VELOCITY;

    private final CANSparkMax tiltMotor;
    private final SparkPIDController tiltController;
    private final RelativeEncoder tiltEncoder;

   
    private final CANSparkMax shooterMotorL; //TOP roller
    private final CANSparkMax shooterMotorR; //BOTTOM roller
    private final SparkPIDController shooterController;

    private final CANSparkMax feedMotor;
    private final RelativeEncoder feedEncoder;

    private final DigitalInput lineBreak;

    private final SysIdRoutine m_sysIdRoutineTilter;
    private final SysIdRoutine m_sysIdRoutineShooterLeft;
    private final SysIdRoutine m_sysIdRoutineShooterRight;

    private final RelativeEncoder shooterMotorREncoder;
    private final RelativeEncoder shooterMotorLEncoder;

    private double tiltPosition = 0.0;

    public Shooter() {
        SHOOTER_FEEDFORWARD_VELOCITY = new SimpleMotorFeedforward(
            0,
            PID_Config.ShooterSubsystem.ShooterVelocityPID.ShooterFeedForward.shooterKV,
            PID_Config.ShooterSubsystem.ShooterVelocityPID.ShooterFeedForward.shooterKA
        );

        tiltMotor = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_TILTER, CANSparkLowLevel.MotorType.kBrushless);
        tiltMotor.setIdleMode(IdleMode.kBrake);
        tiltMotor.setSmartCurrentLimit(45);

        tiltEncoder = tiltMotor.getEncoder();
        tiltEncoder.setPositionConversionFactor(ShooterSubsystemConstants.SHOOTER_TICKS_TO_DEGREES);

        tiltEncoder.setPosition(0.0);

        tiltController = tiltMotor.getPIDController();
        tiltController.setP(TilterPIDConfig.Proportional);
        tiltController.setI(TilterPIDConfig.Integral);
        tiltController.setD(TilterPIDConfig.Derivitive);

        tiltController.setOutputRange(-0.5,0.5);

        shooterMotorL = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_SHOOTER_LEFT, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotorR = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_SHOOTER_RIGHT, CANSparkLowLevel.MotorType.kBrushless);

        shooterMotorL.setSmartCurrentLimit(80);
        shooterMotorR.setSmartCurrentLimit(80);

        shooterMotorL.setInverted(true);
        shooterMotorR.setInverted(false);

        shooterMotorL.setIdleMode(IdleMode.kCoast);
        shooterMotorR.setIdleMode(IdleMode.kCoast);

        shooterController = shooterMotorL.getPIDController();


        shooterController.setP(ShooterVelocityPID.Proportional);
        shooterController.setI(ShooterVelocityPID.Integral);
        shooterController.setD(ShooterVelocityPID.Derivitive);

        //feedMotor = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_FEEDER, CANSparkLowLevel.MotorType.kBrushless);
        feedMotor = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_FEEDER, CANSparkLowLevel.MotorType.kBrushless);

        feedMotor.setSmartCurrentLimit(80);


        feedMotor.setInverted(true);
        feedEncoder = feedMotor.getEncoder();
        feedEncoder.setPositionConversionFactor(ShooterSubsystemConstants.MOTOR_FEEDER_GEARRATIO);
        feedMotor.setIdleMode(IdleMode.kBrake);

        lineBreak = new DigitalInput(0);

        shooterMotorREncoder = shooterMotorR.getEncoder();
        shooterMotorLEncoder = shooterMotorL.getEncoder();

        shooterMotorREncoder.setPositionConversionFactor(ShooterSubsystemConstants.SHOOTER_ROTATIONS_TO_METERS);
        shooterMotorREncoder.setVelocityConversionFactor(ShooterSubsystemConstants.SHOOTER_RPM_TO_MPS);

        shooterMotorLEncoder.setPositionConversionFactor(ShooterSubsystemConstants.SHOOTER_ROTATIONS_TO_METERS);
        shooterMotorLEncoder.setVelocityConversionFactor(ShooterSubsystemConstants.SHOOTER_RPM_TO_MPS);


        // Create a new SysId routine for characterizing the shooter.
        m_sysIdRoutineTilter =
            new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                    // Tell SysId how to plumb the driving voltage to the motor(s).
                    (Measure<Voltage> volts) -> {
                    shooterMotorL.setVoltage(volts.in(Volts));
                    },
                    // Tell SysId how to record a frame of data for each motor on the mechanism being
                    // characterized.
                    log -> {
                        // Record a frame for the shooter motor.
                        log.motor("Shooter Tilter")
                            .voltage(
                                m_appliedVoltage.mut_replace(
                                tiltMotor.getAppliedOutput() * tiltMotor.getBusVoltage(), Volts))
                            .angularPosition(m_angle.mut_replace(tiltEncoder.getPosition(), Degrees))
                            .angularVelocity(
                                m_velocity.mut_replace(tiltEncoder.getVelocity(), DegreesPerSecond));
                    },
                    // Tell SysId to make generated commands require this subsystem, suffix test state in
                    // WPILog with this subsystem's name ("shooter")
                    this
                )
            );
        m_sysIdRoutineShooterLeft = 
            new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                    // Tell SysId how to plumb the driving voltage to the motor(s).
                    (Measure<Voltage> volts) -> {
                    shooterMotorL.setVoltage(volts.in(Volts));
                    },
                    // Tell SysId how to record a frame of data for each motor on the mechanism being
                    // characterized.
                    log -> {
                        // Record a frame for the shooter motor.
                        log.motor("shooter-wheel-left")
                            .voltage(
                                m_appliedVoltage.mut_replace(
                                shooterMotorL.getAppliedOutput() * shooterMotorL.getBusVoltage(), Volts))
                            .angularPosition(m_angle.mut_replace(shooterMotorLEncoder.getPosition(), Rotations))
                            .angularVelocity(
                                m_velocity.mut_replace(shooterMotorLEncoder.getVelocity(), RotationsPerSecond));
                    },
                    // Tell SysId to make generated commands require this subsystem, suffix test state in
                    // WPILog with this subsystem's name ("shooter")
                    this
                )
            );
        m_sysIdRoutineShooterRight = 
            new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                    // Tell SysId how to plumb the driving voltage to the motor(s).
                    (Measure<Voltage> volts) -> {
                    shooterMotorR.setVoltage(volts.in(Volts));
                    },
                    // Tell SysId how to record a frame of data for each motor on the mechanism being
                    // characterized.
                    log -> {
                        // Record a frame for the shooter motor.
                        log.motor("shooter-wheel-right")
                            .voltage(
                                m_appliedVoltage.mut_replace(
                                shooterMotorR.getAppliedOutput() * shooterMotorR.getBusVoltage(), Volts))
                            .angularPosition(m_angle.mut_replace(shooterMotorREncoder.getPosition(), Rotations))
                            .angularVelocity(
                                m_velocity.mut_replace(shooterMotorREncoder.getVelocity(), RotationsPerSecond));
                    },
                    // Tell SysId to make generated commands require this subsystem, suffix test state in
                    // WPILog with this subsystem's name ("shooter")
                    this
                )
            );
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter rollers running in sync", (Math.abs(getShooterFRPM() - getShooterLRPM()) <= 100 )); // Check if shooter rollers are running within 5 RPM of each other
        SmartDashboard.putNumber("Shooter RPM Top", getShooterLRPM());
        SmartDashboard.putNumber("Shooter RPM Bottom", getShooterFRPM());
        SmartDashboard.putNumber("Shooter RPM Difference",Math.abs(getShooterFRPM() - getShooterLRPM()));
        SmartDashboard.putNumber("Tiler Position", getTilterPosition());
        SmartDashboard.putNumber("ShooterCurrentF",shooterMotorR.getOutputCurrent());
        SmartDashboard.putNumber("ShooterCurrentL",shooterMotorL.getOutputCurrent());
        SmartDashboard.putNumber("Tilter Setpoint", tiltPosition);
        SmartDashboard.putBoolean("Shooter Linebreak", getLineBreak());

        //SmartDashboard.putBoolean("Tilter is stuck!", limitSwitchTilter());  
        //SmartDashboard.putString("Shooter Block State", shooterBlock.get().toString());
    }

    public double getShooterLRPM() { 
        return shooterMotorR.getEncoder().getVelocity();
    }
    public double getShooterFRPM() {
        return shooterMotorL.getEncoder().getVelocity();
    }
    public double getFeedPostition() {
        return feedEncoder.getPosition();
    }
    public boolean getLineBreak() {
        return !lineBreak.get();
    }
    public void toggleShooterBlock(DoubleSolenoid.Value value) {
         //shooterBlock.set(value);
    }
    public void setShooterSpeed(double percentOutput) {
        shooterMotorL.set(percentOutput);
        shooterMotorR.set(percentOutput);
    }
    public void setShooterRPM(double desiredRPM) {
        shooterController.setReference(desiredRPM, CANSparkBase.ControlType.kVelocity,0,SHOOTER_FEEDFORWARD_VELOCITY.calculate(desiredRPM));
    }
    public void setFeederSpeed(double percentOutput) {
        feedMotor.set(percentOutput);
    }
    public double getTilterPosition () {
        return tiltEncoder.getPosition();
    }
    public void setTilterPosition(double position) {
        tiltPosition = position;
        tiltController.setReference(position, ControlType.kPosition);
    }
    public void zeroTilterPosition(double newPosition) {
        tiltEncoder.setPosition(newPosition);
    }
    public Command setTilter(double angle) {
        return Commands.runOnce(() -> setTilterPosition(angle), this);
    }
    public Command zeroTilter(double angle) {
        return Commands.runOnce(() -> zeroTilterPosition(angle), this);
    }


    public Command sysIdQuasistaticTilter(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineTilter.quasistatic(direction);
    }
    public Command sysIdDynamicTiler(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineTilter.dynamic(direction);
    }

    
    public Command sysIdQuasistaticLeft(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineShooterLeft.quasistatic(direction);
    }
    public Command sysIdDynamicLeft(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineShooterLeft.dynamic(direction);
    }
    public Command sysIdQuasistaticRight(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineShooterRight.quasistatic(direction);
    }
    public Command sysIdDynamicRight(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineShooterRight.dynamic(direction);
    }
    public Command IdleShooter(){
        return Commands.run(()->setShooterSpeed(0.264),this).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
    }
    
}
