package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.PID_Config.ShooterSubsystem.TilterPIDConfig;
import frc.robot.lib.Constants.ShooterSubsystemConstants;

public class Shooter extends SubsystemBase {


    CANSparkMax tiltMotor;
    SparkPIDController tiltController;

    CANSparkMax shooterMotorL;
    CANSparkMax shooterMotorF;

    CANSparkMax feedMotor;


    public Shooter() {

        tiltMotor = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_TILTER, CANSparkLowLevel.MotorType.kBrushless);
        tiltMotor.setIdleMode(IdleMode.kBrake);

        tiltController = tiltMotor.getPIDController();

        tiltController.setP(TilterPIDConfig.Proportional);
        tiltController.setI(TilterPIDConfig.Integral);
        tiltController.setD(TilterPIDConfig.Derivitive);



        shooterMotorL = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_SHOOTER_LEFT, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotorF = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_SHOOTER_RIGHT, CANSparkLowLevel.MotorType.kBrushless);

        shooterMotorL.setOpenLoopRampRate(ShooterSubsystemConstants.shooterOpenRampRate);
        shooterMotorF.setOpenLoopRampRate(ShooterSubsystemConstants.shooterOpenRampRate);
        shooterMotorL.setInverted(false);
        shooterMotorF.setInverted(true);
        shooterMotorF.follow(shooterMotorL);

        feedMotor = new CANSparkMax(ShooterSubsystemConstants.ID_MOTOR_TILTER, CANSparkLowLevel.MotorType.kBrushless);
        feedMotor.setIdleMode(IdleMode.kBrake);
    }
    
    @Override
    public void periodic() {

    }

    public void setShooterSpeed(double percentOutput) {
        shooterMotorL.set(percentOutput);
    }

    public void setFeederSpeed(double percentOutput) {
        feedMotor.set(percentOutput);
    }
    
    public void setTilterPosition(double position) {
        tiltController.setReference(position, ControlType.kPosition);
    }
    
}
