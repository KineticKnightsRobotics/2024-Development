package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import frc.robot.lib.PID_Config.ShooterSubsystem.TilterPIDConfig;
import frc.robot.lib.Constants.ShooterSubsystemConstants;
import frc.robot.lib.Constants.ShooterSubsystemConstants.ShooterBlockPneumatics;


public class Shooter extends SubsystemBase {


   
    private final CANSparkMax shooterMotorL; //TOP roller
    private final CANSparkMax shooterMotorF; //BOTTOM roller

    private final CANSparkMax feedMotor;

  //  private final Compressor compressor;
   // private final DoubleSolenoid shooterBlock;

    private double tiltPosition = 0.0;


    public Shooter() {

      //  compressor = new Compressor(PneumaticsModuleType.CTREPCM);
       // compressor.enableDigital();

     //  shooterBlock = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ShooterBlockPneumatics.CHANNEL_FORWARD  , ShooterBlockPneumatics.CHANNEL_REVERSE);


   



        shooterMotorL = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_SHOOTER_LEADER, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotorF = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_SHOOTER_FOLLOWER, CANSparkLowLevel.MotorType.kBrushless);

        //shooterMotorL.setOpenLoopRampRate(ShooterSubsystemConstants.shooterOpenRampRate);
        //shooterMotorF.setOpenLoopRampRate(ShooterSubsystemConstants.shooterOpenRampRate);
        shooterMotorL.setInverted(false);
        shooterMotorF.setInverted(true);
        shooterMotorF.follow(shooterMotorL);

        feedMotor = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_TILTER, CANSparkLowLevel.MotorType.kBrushless);
        feedMotor.setIdleMode(IdleMode.kBrake);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter rollers running in sync", (Math.abs(getShooterFRPM() - getShooterLRPM()) <= 5 )); // Check if shooter rollers are running within 5 RPM of each other

        
      //  SmartDashboard.putString("Shooter Block State", shooterBlock.get().toString());
        setShooterSpeed(1);
        setFeederSpeed(1);
    }

    public double getShooterLRPM() { 
        return shooterMotorF.getEncoder().getVelocity();
    }
    public double getShooterFRPM() {
        return shooterMotorL.getEncoder().getVelocity();
    }

  
    
 //   public void toggleShooterBlock(DoubleSolenoid.Value value) {
   //     shooterBlock.set(value);
    //}


    public void setShooterSpeed(double percentOutput) {
        shooterMotorL.set(percentOutput);

    }

    public void setFeederSpeed(double percentOutput) {
        feedMotor.set(percentOutput);
    }

   
    
}
