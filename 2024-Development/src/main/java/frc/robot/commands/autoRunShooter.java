package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Conveyer;

public class autoRunShooter extends Command {
    
    Shooter m_Shooter;
    Conveyer m_Conveyer;

    int desiredRPM = 2500; // magic number :(

    public autoRunShooter(Shooter _shooter, Conveyer _conveyer) {
        addRequirements(_shooter, _conveyer);

        m_Shooter = _shooter;
        m_Conveyer= _conveyer;
    }

    @Override
    public void initialize() {
        m_Shooter.setShooterSpeed(0.8);
        m_Shooter.setFeederSpeed(0.0);
        m_Conveyer.setConveyerSpeed(0.0);        
    }
    @Override
    public void execute() {
        if (m_Shooter.getShooterFRPM() > desiredRPM && m_Shooter.getShooterLRPM() > desiredRPM) {
            m_Shooter.setFeederSpeed(0.8);
        }
    }
    @Override
    public void end(boolean interrupted) {
        m_Shooter.setShooterRPM(0.05);
        m_Shooter.setFeederSpeed(0.0);
        m_Conveyer.setConveyerSpeed(0.0);   
    }
    @Override
    public boolean isFinished() {
       return ! m_Shooter.getLineBreak();
       //return false;
    }
}