package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Conveyer;

public class RUNALLTHEMFELLAS extends Command {


    Shooter m_Shooter;
    Intake m_Intake;
    Conveyer m_Conveyer;

    public RUNALLTHEMFELLAS(Shooter _shooter, Intake _intake, Conveyer _conveyer) {
        addRequirements(_shooter,_conveyer,_intake);

        m_Shooter = _shooter;
        m_Intake = _intake;
        m_Conveyer = _conveyer;
    }

    @Override
    public void initialize() {
        m_Intake.setRollerSpeed(0.8);
        m_Conveyer.setConveyerSpeed(0.4);
        m_Shooter.setFeederSpeed(0.8);
    }

    @Override
    public void end(boolean interrupted) {
        m_Intake.setRollerSpeed(0.0);
        m_Conveyer.setConveyerSpeed(0.0);
        m_Shooter.setFeederSpeed(0.0);
    }


    
}
