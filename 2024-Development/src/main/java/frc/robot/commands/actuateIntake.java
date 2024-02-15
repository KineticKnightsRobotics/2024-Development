package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;

public class INTAKE_actuateIntake extends Command {

    Intake subsystem;
    Double position;

    public INTAKE_actuateIntake(Intake m_subsystem, double intakePosition) {
        subsystem = m_subsystem;
        position = intakePosition;
    }

    @Override
    public void initialize() {
        subsystem.actuateIntake(position);
    }
    @Override
    public void end(boolean Interrupted){

    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
