package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.Intake;


public class INTAKECONVEYER_intakeGamePiece extends Command {

    Intake intake;
    Conveyer conveyer;

    public INTAKECONVEYER_intakeGamePiece(Intake _intake, Conveyer _conveyer){
        addRequirements(_conveyer, _intake);
        intake = _intake;
        conveyer = _conveyer;
    }


    @Override
    public void initialize() {
        intake.setRollerSpeed(1.0);
        conveyer.setConveyerSpeed(0.5);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        intake.setRollerSpeed(0.0);
        conveyer.setConveyerSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        //return conveyer.getLineBreak();
        return false;
    }
}
