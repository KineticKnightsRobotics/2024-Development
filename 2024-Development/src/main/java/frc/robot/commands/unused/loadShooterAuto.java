package frc.robot.commands.unused;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.Shooter;

public class loadShooterAuto extends Command {
    

    Shooter m_Shooter;
    Conveyer m_Conveyer;

    public loadShooterAuto(Conveyer _Conveyer, Shooter _Shooter) {
        addRequirements(_Conveyer,_Shooter);
        m_Shooter=_Shooter;
        m_Conveyer=_Conveyer;
    } 
    @Override
    public void initialize() {
        m_Shooter.setFeederSpeed(-0.5);
        m_Conveyer.setConveyerSpeed(0.4);
    }
    @Override
    public void end(boolean interrupted) {
        m_Shooter.setFeederSpeed(0.0);
        m_Conveyer.setConveyerSpeed(0.0);

    }
    @Override
    public boolean isFinished() {
        return m_Shooter.getLineBreak();
    }
    @Override
    public void execute(){
        m_Shooter.setFeederSpeed(-0.5);
        m_Conveyer.setConveyerSpeed(0.4);
    }
}
