package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.lib.Constants.ClimberSubsystemConstants;

public class Climber extends SubsystemBase {
    
    CANSparkMax leftWinch;
    CANSparkMax rightWinch;

    double kP = -0.1;

    public Climber() {
        leftWinch = new CANSparkMax(ClimberSubsystemConstants.ID_LEFT_WINCH, MotorType.kBrushless);
        rightWinch= new CANSparkMax(ClimberSubsystemConstants.ID_RIGHT_WINCH,MotorType.kBrushless);

        leftWinch.setSmartCurrentLimit(60);
        rightWinch.setSmartCurrentLimit(60);

        leftWinch.setIdleMode(IdleMode.kBrake);
        rightWinch.setIdleMode(IdleMode.kBrake);

        leftWinch.setInverted(false);
        rightWinch.setInverted(true);
    }

    public void setWinchOutput(double percentOutput) {
       leftWinch.set(percentOutput);
       rightWinch.set(percentOutput);
    }

    /*
    public Command setWinchSpeed(double percentOutput) {
        return Commands.run(() -> setWinchOutput(percentOutput));
    }
    */


    public Command setWinchSpeed(double percentOutput) {
        return Commands.runOnce(() -> {
            leftWinch.set(percentOutput);
            rightWinch.set(percentOutput);
        },this).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command climberUp() {
        return Commands.
        run( () -> {
            leftWinch .set(rightWinch.getEncoder().getPosition() - ClimberSubsystemConstants.climberUpPosition * kP * 0.2);
            rightWinch.set(rightWinch.getEncoder().getPosition() - ClimberSubsystemConstants.climberUpPosition * kP * 0.2);
        })
        .until(() -> Math.abs(leftWinch.getEncoder().getPosition() - ClimberSubsystemConstants.climberUpPosition) <= 10 && Math.abs(rightWinch.getEncoder().getPosition() - ClimberSubsystemConstants.climberUpPosition) <=10)
        .finallyDo(() -> {
            leftWinch.set(0.0);
            rightWinch.set(0.0);
        });
    }
}
