package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.lib.Constants.ConveyerSubsystemConstants;

public class Conveyer extends SubsystemBase {

    CANSparkMax conveyerMotorLeft;
    CANSparkMax conveyerMotorRight;


    
    public Conveyer() {
        conveyerMotorLeft = new CANSparkMax(ConveyerSubsystemConstants.ID_MOTOR_CONVEYER_LEFT, CANSparkLowLevel.MotorType.kBrushless);
        conveyerMotorRight = new CANSparkMax(ConveyerSubsystemConstants.ID_MOTOR_CONVEYER_RIGHT, CANSparkLowLevel.MotorType.kBrushless);
        
        conveyerMotorLeft.setOpenLoopRampRate(2);
        conveyerMotorRight.setOpenLoopRampRate(2);
    }

    @Override
    public void periodic() {

    }

    public void setConveyerSpeed(double percentOutput) {
        conveyerMotorRight.set(percentOutput);
        conveyerMotorLeft.set(-percentOutput);
    }

    public Command runConveyer(double percentOutput) {
        return Commands.runOnce(() -> setConveyerSpeed(percentOutput), this);
    }


}
