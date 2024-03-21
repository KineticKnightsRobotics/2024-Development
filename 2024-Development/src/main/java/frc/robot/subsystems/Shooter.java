package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.lib.PID_Config.ShooterSubsystem.ShooterVelocityPID;
import frc.robot.lib.PID_Config.ShooterSubsystem.TilterPIDConfig;
import frc.robot.lib.PID_Config.ShooterSubsystem.ExtensionPID;
import frc.robot.lib.Constants.ShooterSubsystemConstants;
import frc.robot.lib.ShooterInterpolator;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class Shooter extends SubsystemBase {

    private final SimpleMotorFeedforward shooterFeedFoward;
    public final CANSparkMax tiltMotor;
    private final CANSparkMax tiltMotor_Follower;
    public final PIDController tiltControllerHome;
    public final PIDController tiltControllerExtend;

    public final ProfiledPIDController tiltTrapezoidProfile;


    private final RelativeEncoder tiltEncoder;
    private final DigitalInput tiltLimitSwitch;


    private final CANSparkMax shooterMotorL; //TOP roller
    private final CANSparkMax shooterMotorR; //BOTTOM roller
    private final SparkPIDController shooterControllerL;
    private final SparkPIDController shooterControllerR;
    private final RelativeEncoder shooterMotorREncoder;
    private final RelativeEncoder shooterMotorLEncoder;

    private final CANSparkMax feedMotor;
    private final RelativeEncoder feedEncoder;
    private final DigitalInput lineBreak;
    private final double tiltPosition = 0.0;
    
    public final ShooterInterpolator shooterInterpolator;
    private final CANSparkMax extensionMotor;
    private final RelativeEncoder extensionEncoder;
    public final SparkPIDController extensionController;

    /*
    private final SysIdRoutine m_sysIdRoutineTilter;
    private final SysIdRoutine m_sysIdRoutineShooterLeft;
    private final SysIdRoutine m_sysIdRoutineShooterRight;
    */


    public Shooter() {


        shooterInterpolator = new ShooterInterpolator();
        
        tiltMotor = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_TILTER, CANSparkLowLevel.MotorType.kBrushless);
        tiltMotor.setIdleMode(IdleMode.kBrake);
        tiltMotor.setSmartCurrentLimit(50);
        tiltMotor_Follower = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_TILTER_FOLLOWER, CANSparkLowLevel.MotorType.kBrushless);
        tiltMotor_Follower.setIdleMode(IdleMode.kBrake);
        tiltMotor_Follower.setSmartCurrentLimit(45);
        tiltMotor.setInverted(true);
        tiltMotor_Follower.setInverted(false);
        tiltMotor_Follower.follow(tiltMotor);

        tiltEncoder = tiltMotor.getEncoder();
        tiltEncoder.setPositionConversionFactor(ShooterSubsystemConstants.SHOOTER_TICKS_TO_DEGREES);
        tiltEncoder.setPosition(0.0);

        tiltLimitSwitch = new DigitalInput(9);

        tiltControllerHome = new PIDController(
            TilterPIDConfig.home.Proportional,
            TilterPIDConfig.home.Integral,
            TilterPIDConfig.home.Derivitive
        );

        tiltControllerExtend = new PIDController(
            TilterPIDConfig.extended.Proportional,
            TilterPIDConfig.extended.Integral,
            TilterPIDConfig.extended.Derivitive
        );

        tiltTrapezoidProfile = new ProfiledPIDController(
            TilterPIDConfig.extended.Proportional,
            TilterPIDConfig.extended.Integral,
            TilterPIDConfig.extended.Derivitive, 
            new TrapezoidProfile.Constraints(
                900,
                1200
            )
        );


        shooterMotorL = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_SHOOTER_LEFT, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotorR = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_SHOOTER_RIGHT, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotorL.setSmartCurrentLimit(80);
        shooterMotorR.setSmartCurrentLimit(80);
        shooterMotorL.setInverted(true);
        shooterMotorR.setInverted(false);
        shooterMotorL.setIdleMode(IdleMode.kCoast);
        shooterMotorR.setIdleMode(IdleMode.kCoast);

        shooterMotorREncoder = shooterMotorR.getEncoder();
        shooterMotorLEncoder = shooterMotorL.getEncoder();

        shooterMotorLEncoder.setVelocityConversionFactor(1/60);
        shooterMotorREncoder.setVelocityConversionFactor(1/60);

        shooterControllerL = shooterMotorL.getPIDController();
        shooterControllerL.setP(ShooterVelocityPID.Proportional);
        shooterControllerL.setI(ShooterVelocityPID.Integral);
        shooterControllerL.setD(ShooterVelocityPID.Derivitive);

        shooterControllerR = shooterMotorR.getPIDController();
        shooterControllerR.setP(ShooterVelocityPID.Proportional);
        shooterControllerR.setI(ShooterVelocityPID.Integral);
        shooterControllerR.setD(ShooterVelocityPID.Derivitive);

        shooterFeedFoward = new SimpleMotorFeedforward(
            ShooterVelocityPID.ShooterFeedForward.shooterKS,
            ShooterVelocityPID.ShooterFeedForward.shooterKV,
            ShooterVelocityPID.ShooterFeedForward.shooterKA
        );


        //feedMotor = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_FEEDER, CANSparkLowLevel.MotorType.kBrushless);
        feedMotor = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_FEEDER, CANSparkLowLevel.MotorType.kBrushless);
        feedMotor.setSmartCurrentLimit(80);


        feedMotor.setInverted(false);
        feedEncoder = feedMotor.getEncoder();
        feedEncoder.setPositionConversionFactor(ShooterSubsystemConstants.MOTOR_FEEDER_GEARRATIO);
        feedMotor.setIdleMode(IdleMode.kBrake);

        lineBreak = new DigitalInput(0);

        shooterMotorREncoder.setPositionConversionFactor(1);
        shooterMotorREncoder.setVelocityConversionFactor(1);

        shooterMotorLEncoder.setPositionConversionFactor(1);
        shooterMotorLEncoder.setVelocityConversionFactor(1);

        //SmartDashboard.putNumber("Manual Shooter Angle", tiltPosition);     

        extensionMotor = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_EXTENSION, MotorType.kBrushless);

        extensionMotor.setIdleMode(IdleMode.kBrake);
        extensionMotor.setInverted(true);
        extensionMotor.setSmartCurrentLimit(40);

        extensionEncoder = extensionMotor.getEncoder();
        extensionEncoder.setPositionConversionFactor(ShooterSubsystemConstants.EXTENSION_ROT_TO_HEIGHT);

        extensionController = extensionMotor.getPIDController();

        extensionController.setP(ExtensionPID.Proportional);
        extensionController.setI(ExtensionPID.Integral);
        extensionController.setD(ExtensionPID.Derivitive);

    }
    
    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("Shooter Extension Position", extensionEncoder.getPosition());

        SmartDashboard.putNumber("Shooter Tilter Position", tiltEncoder.getPosition());

        //SmartDashboard.putNumber("Through Bore Encoder Absolute", throughBoreEncoder.getAbsolutePosition());

        SmartDashboard.putNumber("Shooter RPM Left", shooterMotorLEncoder.getVelocity());

        SmartDashboard.putNumber("Shooter RPM Right", shooterMotorREncoder.getVelocity());

        //SmartDashboard.putNumber("ShooterCurrentF",shooterMotorR.getOutputCurrent());
        //SmartDashboard.putNumber("ShooterCurrentL",shooterMotorL.getOutputCurrent());

        SmartDashboard.putBoolean("Shooter Linebreak", getLineBreak());


        SmartDashboard.putBoolean("AAAA Limit Switch", tiltLimitSwitch.get());
    }

    public boolean getLineBreak() {
        return !lineBreak.get();
    }

    public double getExtensionPosition() {
        return extensionEncoder.getPosition();
    }

    public double getTilterPosition () {
        return tiltEncoder.getPosition();
    }
    
    public boolean ampPostion() {
        return extensionEncoder.getPosition() > 2.0;
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
        double leftSpeed = desiredRPM_1/60;
        double rightSpeed= desiredRPM_2/60;

        return Commands
        .run(
            () -> {
                if (openLoop) {
                    shooterMotorL.set(1.0);
                    shooterMotorR.set(1.0);
                }
                else {
                shooterControllerL.setReference(leftSpeed, ControlType.kVelocity,0,shooterFeedFoward.calculate(leftSpeed), ArbFFUnits.kVoltage);
                shooterControllerR.setReference(rightSpeed, ControlType.kVelocity,0,shooterFeedFoward.calculate(rightSpeed), ArbFFUnits.kVoltage);
                }
                if (shooterMotorLEncoder.getVelocity() >= desiredRPM_1-20 && shooterMotorREncoder.getVelocity() >= desiredRPM_2-20){
                    feedMotor.set(1.0); 
                }
            },
        this)
        .until(() -> ! getLineBreak())
        .andThen(new WaitCommand(0.25))
        .finallyDo(
            () -> {
                shooterMotorL.set(0.0);
                shooterMotorR.set(0.0);
                feedMotor.set(0.0);
            }
        );
    }

    public Command spitOutNote() {
        return Commands
            .run(
                () -> {
                    shooterMotorL.set(0.2);
                    shooterMotorR.set(0.2);
                    feedMotor.set(0.5);
                }
            )
            .until(() -> ! getLineBreak())
            .andThen(new WaitCommand(0.25))
            .finallyDo(
                () -> {
                    shooterMotorL.set(0.0);
                    shooterMotorR.set(0.0);
                    feedMotor.set(0.0);
                }
            );
    }

    public Command setTilter(DoubleSupplier angle) {
        return Commands.run(
            () -> {
                //tiltMotor.set(MathUtil.clamp(tiltControllerExtend.calculate(tiltEncoder.getPosition(), angle.getAsDouble()),-0.5,0.5));
                tiltMotor.set(MathUtil.clamp(tiltTrapezoidProfile.calculate(tiltEncoder.getPosition(), angle.getAsDouble()), -0.5, 0.5));
            }
        ).finallyDo(
           () -> {
                tiltMotor.stopMotor();
           }
        );
    }

    public Command stopTilter(){
        return Commands.runOnce(
            ()->{
                tiltMotor.set(0);
            }
            );
    }

    public Command setTilterVoltage(double Voltage) {
        return Commands.run(
            () -> {
                tiltMotor.setVoltage(Voltage);
            }
        ).finallyDo(
            () -> {
                tiltMotor.setVoltage(0.0);
            }
        );
    }

    public Command zeroTilter(double angle) {
        //return Commands.runOnce(() -> {tiltEncoder.setPosition(angle);});
                return Commands.runOnce(() -> {tiltEncoder.setPosition(0.0);});

    }
    
    public Command IdleShooter(double idleRPM_L,double idleRPM_R){
        return Commands
        .runOnce(
            ()->{
                shooterMotorL.set(0.3);
                shooterMotorR.set(0.3);
            }
        )
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
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
                //feedMotor.set(0.0);
            }
        )
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command reverseShooter() {
        return Commands
            .runOnce(
                () -> {
                    shooterMotorL.set(-0.1);
                    shooterMotorR.set(-0.1);
                }
            );
    }

    public Command stopShooter() {
        return Commands.
            runOnce(
                () -> {
                    shooterMotorL.stopMotor();
                    shooterMotorR.stopMotor();
                }
            );
    }

    public Command setFeederSpeed(double percentOutput) {
        return Commands.runOnce(() -> {feedMotor.set(percentOutput);});
    }

    public Command setExtensionSpeed(double percentOutput) {
        return Commands.runOnce(() -> {extensionMotor.set(percentOutput);});
    }

    public Command setExtensionHeight(double height) {
        return Commands.runOnce(() -> {extensionController.setReference(height, ControlType.kPosition);});
    }   
}
