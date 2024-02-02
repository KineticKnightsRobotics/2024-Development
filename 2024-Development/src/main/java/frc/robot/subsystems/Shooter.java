package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
//import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.lib.PID_Config.ShooterSubsystem.TilterPIDConfig;
//import frc.robot.lib.PID_Config.ShooterSubsystem.TilterPIDConfig;
import frc.robot.lib.Constants.ShooterSubsystemConstants;
import frc.robot.lib.Constants.ShooterSubsystemConstants.ShooterBlockPneumatics;


public class Shooter extends SubsystemBase {


    private final CANSparkMax tiltMotor;
    private final SparkPIDController tiltController;
    private final RelativeEncoder tiltEncoder;

   
    private final CANSparkMax shooterMotorL; //TOP roller
    private final CANSparkMax shooterMotorF; //BOTTOM roller

    private final CANSparkMax feedMotor;
    private final RelativeEncoder feedEncoder;

    //private final Compressor compressor;
    //private final DoubleSolenoid shooterBlock;

    private double tiltPosition = 0.0;


    public Shooter() {

        tiltMotor = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_TILTER, CANSparkLowLevel.MotorType.kBrushless);
        tiltMotor.setIdleMode(IdleMode.kBrake);

        tiltEncoder = tiltMotor.getEncoder();
        //tiltEncoder.setPositionConversionFactor(); TODO: Find this!

        tiltController = tiltMotor.getPIDController();
        tiltController.setP(TilterPIDConfig.Proportional);
        tiltController.setI(TilterPIDConfig.Integral);
        tiltController.setD(TilterPIDConfig.Derivitive);


        //Pneumatics stuff is not on the robot yet...
        //compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        //compressor.enableDigital();
        //shooterBlock = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ShooterBlockPneumatics.CHANNEL_FORWARD  , ShooterBlockPneumatics.CHANNEL_REVERSE);


        shooterMotorL = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_SHOOTER_LEADER, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotorF = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_SHOOTER_FOLLOWER, CANSparkLowLevel.MotorType.kBrushless);

        shooterMotorL.setInverted(false);
        shooterMotorF.setInverted(true);
        shooterMotorF.follow(shooterMotorL);

        feedMotor = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_FEEDER, CANSparkLowLevel.MotorType.kBrushless);
        feedEncoder = feedMotor.getEncoder();
        feedEncoder.setPositionConversionFactor(ShooterSubsystemConstants.MOTOR_FEEDER_GEARRATIO);
        feedMotor.setIdleMode(IdleMode.kBrake);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter rollers running in sync", (Math.abs(getShooterFRPM() - getShooterLRPM()) <= 100 )); // Check if shooter rollers are running within 5 RPM of each other

        SmartDashboard.putNumber("Shooter RPM Top", getShooterLRPM());
        SmartDashboard.putNumber("Shooter RPM Bottom", getShooterFRPM());

        //SmartDashboard.putBoolean("Tilter is stuck!", limitSwitchTilter());  

        //SmartDashboard.putString("Shooter Block State", shooterBlock.get().toString());
    }

    public double getShooterLRPM() { 
        return shooterMotorF.getEncoder().getVelocity();
    }
    public double getShooterFRPM() {
        return shooterMotorL.getEncoder().getVelocity();
    }

    public double getFeedPostition() {
        return feedEncoder.getPosition();
    }
  
    
    public void toggleShooterBlock(DoubleSolenoid.Value value) {
         //shooterBlock.set(value);
    }


    public void setShooterSpeed(double percentOutput) {
        shooterMotorL.set(percentOutput);
    }

    public void setFeederSpeed(double percentOutput) {
        feedMotor.set(percentOutput);
    }

    /*
    public void setTilterPosition(double position) {
        tiltPosition = position;
        tiltController.setReference(position, ControlType.kPosition);
    }
    */
    
}
