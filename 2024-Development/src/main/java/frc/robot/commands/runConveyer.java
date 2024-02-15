package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Conveyer;

public class runConveyer extends Command {


    Conveyer m_Conveyer;
    double percentOutput;

    public runConveyer(double new_speed, Conveyer subsystem) {
        addRequirements(subsystem);
 
        m_Conveyer = subsystem;
        percentOutput = new_speed;
    }

    @Override
    public void initialize() {
        m_Conveyer.setConveyerSpeed(percentOutput);
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
