package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.lib.Constants.ConveyerSubsystemConstants;

public class Conveyer extends SubsystemBase {

    CANSparkMax conveyerMotor;


    
    public Conveyer() {
        conveyerMotor = new CANSparkMax(ConveyerSubsystemConstants.ID_MOTOR_CONVEYER, CANSparkLowLevel.MotorType.kBrushless);

        conveyerMotor.setOpenLoopRampRate(2);
    }

    @Override
    public void periodic() {

    }

    public void setConveyerSpeed(double percentOutput) {
        conveyerMotor.set(percentOutput);
    }

    public Command runConveyer(double percentOutput) {
        return Commands.runOnce(() -> setConveyerSpeed(percentOutput), this);
    }


}
