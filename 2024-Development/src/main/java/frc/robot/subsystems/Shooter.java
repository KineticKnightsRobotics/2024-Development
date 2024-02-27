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

     private final SysIdRoutine m_sysIdRoutineShooterLeader;

     RelativeEncoder shooterMotorREncoder;
     RelativeEncoder shooterMotorLEncoder;

    //private final Compressor compressor;
    //private final DoubleSolenoid shooterBlock;

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
        tiltEncoder.setPositionConversionFactor(ShooterSubsystemConstants.SHOOTER_TICKS_TO_DEGREES); //TODO: Find this!

        tiltEncoder.setPosition(0.0);

        tiltController = tiltMotor.getPIDController();
        tiltController.setP(TilterPIDConfig.Proportional);
        tiltController.setI(TilterPIDConfig.Integral);
        tiltController.setD(TilterPIDConfig.Derivitive);

        tiltController.setOutputRange(-0.5,0.5);


        //Pneumatics stuff is not on the robot yet...
        //compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        //compressor.enableDigital();
        //shooterBlock = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ShooterBlockPneumatics.CHANNEL_FORWARD  , ShooterBlockPneumatics.CHANNEL_REVERSE);


        shooterMotorL = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_SHOOTER_LEFT, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotorR = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_SHOOTER_RIGHT, CANSparkLowLevel.MotorType.kBrushless);

        //shooterMotorL.restoreFactoryDefaults();
        //shooterMotorF.restoreFactoryDefaults();

        shooterMotorL.setSmartCurrentLimit(80);
        shooterMotorR.setSmartCurrentLimit(80);

        shooterMotorL.setInverted(true);
        shooterMotorR.setInverted(false);
        //shooterMotorL.follow(shooterMotorF);

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


 // Create a new SysId routine for characterizing the shooter.
  m_sysIdRoutineShooterLeader = 
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
           log.motor("shooter-wheel-leader")
               .voltage(
                   m_appliedVoltage.mut_replace(
                       shooterMotorL.getAppliedOutput() * shooterMotorL.getBusVoltage(), Volts))
               .angularPosition(m_angle.mut_replace(shooterMotorLEncoder.getPosition(), Rotations))
               .angularVelocity(
                   m_velocity.mut_replace(shooterMotorLEncoder.getVelocity(), RotationsPerSecond));
         },
         // Tell SysId to make generated commands require this subsystem, suffix test state in
         // WPILog with this subsystem's name ("shooter")
         this));
        
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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineShooterLeader.quasistatic(direction);
      }
    
      /**
       * Returns a command that will execute a dynamic test in the given direction.
       *
       * @param direction The direction (forward or reverse) to run the test in
       */
      public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineShooterLeader.dynamic(direction);
      }
    
}
