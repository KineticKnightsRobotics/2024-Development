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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.lib.PID_Config.ShooterSubsystem.ShooterVelocityPID;
import frc.robot.lib.PID_Config.ShooterSubsystem.TilterPIDConfig;
//import frc.robot.lib.PID_Config.ShooterSubsystem.TilterPIDConfig;
import frc.robot.lib.Constants.ShooterSubsystemConstants;
import frc.robot.lib.PID_Config;
import frc.robot.lib.ShooterInterpolator;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.MutableMeasure.mutable;
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
    //private final MutableMeasure<Angle> m_degrees = mutable(Degrees.of(0));
    //private final MutableMeasure<Velocity<Angle>> m_velocity_degrees = mutable(DegreesPerSecond.of(0));
    private final SimpleMotorFeedforward shooterFeedFoward;
    private final CANSparkMax tiltMotor;
    private final CANSparkMax tiltMotor_Follower;
    private final SparkPIDController tiltController;
    private final RelativeEncoder tiltEncoder;
    private final CANSparkMax shooterMotorL; //TOP roller
    private final CANSparkMax shooterMotorR; //BOTTOM roller
    private final SparkPIDController shooterControllerL;
    private final SparkPIDController shooterControllerR;
    private final CANSparkMax feedMotor;
    private final RelativeEncoder feedEncoder;
    private final DigitalInput lineBreak;
    private final SysIdRoutine m_sysIdRoutineTilter;
    private final SysIdRoutine m_sysIdRoutineShooterLeft;
    private final SysIdRoutine m_sysIdRoutineShooterRight;
    private final RelativeEncoder shooterMotorREncoder;
    private final RelativeEncoder shooterMotorLEncoder;
    private double tiltPosition = 0.0;
    public final ShooterInterpolator shooterInterpolator = new ShooterInterpolator();
    public Shooter() {
        shooterFeedFoward = new SimpleMotorFeedforward(
            PID_Config.ShooterSubsystem.ShooterVelocityPID.ShooterFeedForward.shooterKS,
            PID_Config.ShooterSubsystem.ShooterVelocityPID.ShooterFeedForward.shooterKV,
            PID_Config.ShooterSubsystem.ShooterVelocityPID.ShooterFeedForward.shooterKA
        );
        tiltMotor = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_TILTER, CANSparkLowLevel.MotorType.kBrushless);
        tiltMotor.setIdleMode(IdleMode.kBrake);
        tiltMotor.setSmartCurrentLimit(45);
        tiltMotor_Follower = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_TILTER_FOLLOWER, CANSparkLowLevel.MotorType.kBrushless);
        tiltMotor_Follower.setIdleMode(IdleMode.kBrake);
        tiltMotor_Follower.setSmartCurrentLimit(45);
        tiltMotor.setInverted(true);
        tiltMotor_Follower.setInverted(false);
        tiltMotor_Follower.follow(tiltMotor);

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

        shooterControllerL = shooterMotorL.getPIDController();
        shooterControllerL.setP(ShooterVelocityPID.Proportional);
        shooterControllerL.setI(ShooterVelocityPID.Integral);
        shooterControllerL.setD(ShooterVelocityPID.Derivitive);

        shooterControllerR = shooterMotorR.getPIDController();
        shooterControllerR.setP(ShooterVelocityPID.Proportional);
        shooterControllerR.setI(ShooterVelocityPID.Integral);
        shooterControllerR.setD(ShooterVelocityPID.Derivitive);

        //feedMotor = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_FEEDER, CANSparkLowLevel.MotorType.kBrushless);
        feedMotor = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_FEEDER, CANSparkLowLevel.MotorType.kBrushless);
        feedMotor.setSmartCurrentLimit(80);


        feedMotor.setInverted(false);
        feedEncoder = feedMotor.getEncoder();
        feedEncoder.setPositionConversionFactor(ShooterSubsystemConstants.MOTOR_FEEDER_GEARRATIO);
        feedMotor.setIdleMode(IdleMode.kBrake);

        lineBreak = new DigitalInput(0);

        shooterMotorREncoder = shooterMotorR.getEncoder();
        shooterMotorLEncoder = shooterMotorL.getEncoder();

        shooterMotorREncoder.setPositionConversionFactor(1);
        shooterMotorREncoder.setVelocityConversionFactor(1);

        shooterMotorLEncoder.setPositionConversionFactor(1);
        shooterMotorLEncoder.setVelocityConversionFactor(1);

        SmartDashboard.putNumber("Manual Shooter Angle", tiltPosition);

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
        //SmartDashboard.putBoolean("Shooter rollers running in sync", (Math.abs(getShooterFRPM() - getShooterLRPM()) <= 100 )); // Check if shooter rollers are running within 5 RPM of each other
        //SmartDashboard.putNumber("Shooter RPM Top", shooterMotorR.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter RPM Bottom", shooterMotorL.getEncoder().getVelocity());
        //SmartDashboard.putNumber("Shooter RPM Difference",Math.abs(getShooterFRPM() - getShooterLRPM()));
        SmartDashboard.putNumber("Tiler Position", getTilterPosition());
        //SmartDashboard.putNumber("ShooterCurrentF",shooterMotorR.getOutputCurrent());
        //SmartDashboard.putNumber("ShooterCurrentL",shooterMotorL.getOutputCurrent());
        //SmartDashboard.putNumber("Tilter Setpoint", tiltPosition);
        SmartDashboard.putBoolean("Shooter Linebreak", getLineBreak());
        //SmartDashboard.putBoolean("Tilter is stuck!", limitSwitchTilter());  
        //SmartDashboard.putString("Shooter Block State", shooterBlock.get().toString());
    }
    public boolean getLineBreak() {
        return !lineBreak.get();
    }
    public double getTilterPosition () {
        return tiltEncoder.getPosition();
    }

    public Command setTilter(double angle) {
        return Commands.runOnce(
            () -> {
                tiltController.setReference(angle, ControlType.kPosition);
            }
        );
    }
    public Command setTiltertoManual() {
        if (SmartDashboard.getNumber("Manual Shooter Angle",0.0) != tiltPosition) {
            tiltPosition = SmartDashboard.getNumber("Manual Shooter Angle", 0.0);
        }
        SmartDashboard.putNumber("Current Manual Positon", tiltPosition);
        return Commands.runOnce(
            () -> {
                tiltController.setReference(tiltPosition, ControlType.kPosition);
            }
        );
    }

    public Command zeroTilter(double angle) {
        return Commands.runOnce(() -> {tiltEncoder.setPosition(angle);});
    }
    public Command IdleShooter(){
        return Commands
        .run(
            ()->{
                shooterMotorL.set(0.264);
                shooterMotorR.set(0.264);
            }
        )
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
    }

    /**
     * Sets Flywheel speed speed. 2 Speeds don't work right in open loop mode, higher RPM will supersede the lower RPM
     * 
     * @param desiredRPM_1 Left Side
     * @param desiredRPM_2 Right Side
     * @param openLoop
     * @return
     */
    public Command shoot(double desiredRPM_1, double desiredRPM_2, boolean openLoop) {
        return Commands
        .run(
            () -> {

                if (openLoop) {
                    shooterMotorL.set(1.0);
                    shooterMotorR.set(1.0);
                }
                else {
                shooterControllerL.setReference(desiredRPM_1, ControlType.kVelocity,0,shooterFeedFoward.calculate(desiredRPM_1));
                shooterControllerR.setReference(desiredRPM_2, ControlType.kVelocity,0,shooterFeedFoward.calculate(desiredRPM_2));
                }
                if (shooterMotorLEncoder.getVelocity() >= desiredRPM_1-20 && shooterMotorREncoder.getVelocity() >= desiredRPM_2-20){
                    feedMotor.set(1.0);
                }
            },
        this)
        .until(() -> ! getLineBreak())
        .andThen(new WaitCommand(1.0))
        .finallyDo(
            () -> {
                shooterMotorL.set(0.0);
                shooterMotorR.set(0.0);
                feedMotor.set(0.0);
            }
        );
    }

    public Command loadGamePiece() {
        return Commands
        .run(
            () -> {
                feedMotor.set(0.4);
            }
        ,this)
        .until(() -> getLineBreak())
        .finallyDo(
            () -> {
                feedMotor.set(0.0);
            }
        )
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }
    public Command setFeederSpeed(double percentOutput) {
        return Commands.runOnce(() -> {feedMotor.set(percentOutput);},this);
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
    
}
