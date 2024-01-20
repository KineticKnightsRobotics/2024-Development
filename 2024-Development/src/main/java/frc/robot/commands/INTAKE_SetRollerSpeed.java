package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;

public class INTAKE_SetRollerSpeed extends Command {

    Intake subsystem;
    Double percentOutput;

    public INTAKE_SetRollerSpeed(Intake m_subsystem, double output) {
        subsystem = m_subsystem;
        percentOutput = output;
    }

    @Override
    public void initialize() {
        subsystem.setRollerSpeed(percentOutput);
    }
    @Override
    public void end(boolean Interrupted){
        if (Interrupted){subsystem.setRollerSpeed(0.0);}
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
