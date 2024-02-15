package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class autoSetShooterIdle extends Command {
    

    Shooter m_Shooter;

    public autoSetShooterIdle() {
        
    }
    @Override
    public void initialize() {

    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
