package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.Constants.ClimberSubsystemConstants;

public class Climber extends SubsystemBase {
    
    CANSparkMax leftWinch;
    CANSparkMax rightWinch;

    public Climber() {
        leftWinch = new CANSparkMax(ClimberSubsystemConstants.ID_LEFT_WINCH, MotorType.kBrushless);
        rightWinch= new CANSparkMax(ClimberSubsystemConstants.ID_RIGHT_WINCH,MotorType.kBrushless);

        leftWinch.setSmartCurrentLimit(60);
        rightWinch.setSmartCurrentLimit(60);

        leftWinch.setIdleMode(IdleMode.kBrake);
        rightWinch.setIdleMode(IdleMode.kBrake);

        leftWinch.setInverted(false);
        rightWinch.setInverted(false);

   //  leftWinch.follow(rightWinch);

    }

    public void setWinchOutput(double percentOutput) {
       leftWinch.set(percentOutput);
       rightWinch.set(percentOutput);
    }

    public Command setWinchSpeed(double percentOutput) {
        return Commands.run(() -> setWinchOutput(percentOutput));
    }
}
