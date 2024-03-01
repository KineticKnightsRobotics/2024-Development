package frc.robot.commands.unused;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;


public class runFeeder extends Command {


    Shooter m_Shooter;

    double m_Speed;


    public runFeeder(double speed, Shooter subsystem) {
        addRequirements(subsystem);

        m_Shooter = subsystem;
        m_Speed = speed;
    }

    @Override
    public void initialize() {
        m_Shooter.setFeederSpeed(m_Speed);
    }
    @Override
    public void end(boolean interrupted) {
        m_Shooter.setFeederSpeed(0.0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
