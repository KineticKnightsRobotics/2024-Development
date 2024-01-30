package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Conveyer;

public class CONVEYER_RunConveyer extends Command {


    Conveyer m_Conveyer;


    public CONVEYER_RunConveyer(double speed, Conveyer subsystem) {
        addRequirements(subsystem);

        m_Conveyer = subsystem;
    }

    @Override
    public void initialize() {
        m_Conveyer.setConveyerSpeed(0.4);
    }

    @Override
    public void end(boolean interrupted) {
        m_Conveyer.setConveyerSpeed(0.0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }

    
}
