package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.Shooter;

public class loadFeederToggle extends Command {
    

    Shooter m_Shooter;
    Conveyer m_Conveyer;

    public loadFeederToggle(Conveyer _Conveyer, Shooter _Shooter) {
        addRequirements(_Conveyer,_Shooter);
        m_Shooter=_Shooter;
        m_Conveyer=_Conveyer;
    } 
    @Override
    public void initialize() {
        m_Shooter.setFeederSpeed(-0.5);
        m_Conveyer.setConveyerSpeed(0.5);
    }
    @Override
    public void end(boolean interrupted) {
        m_Shooter.setFeederSpeed(0.0);
        m_Conveyer.setConveyerSpeed(0);
        m_Conveyer.toggle(false);
    }
    @Override
    public boolean isFinished() {
        return m_Shooter.getLineBreak();
    }
    @Override
    public void execute(){
        m_Shooter.setFeederSpeed(-0.5);
                m_Conveyer.setConveyerSpeed(0.5);

    }
}
