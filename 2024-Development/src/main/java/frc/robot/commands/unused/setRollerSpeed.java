package frc.robot.commands.unused;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;

public class setRollerSpeed extends Command {

    Intake subsystem;
    Double percentOutput;

    public setRollerSpeed(Intake m_subsystem, double output) {
        subsystem = m_subsystem;
        percentOutput = output;
    }

    @Override
    public void initialize() {
        subsystem.setRollerSpeed(percentOutput);
    }
    @Override
    public void end(boolean Interrupted){
        subsystem.setRollerSpeed(0.0);
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
