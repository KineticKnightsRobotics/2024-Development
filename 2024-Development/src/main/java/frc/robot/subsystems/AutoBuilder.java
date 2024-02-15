package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LimeLight;



public class AutoBuilder extends SubsystemBase {
    

    SwerveDrive m_Swerve;
    LimeLight m_LimeLight;

    public AutoBuilder(SwerveDrive _swerve, LimeLight _limelight) {
        m_Swerve = _swerve;
        m_LimeLight = _limelight;
    }
}
