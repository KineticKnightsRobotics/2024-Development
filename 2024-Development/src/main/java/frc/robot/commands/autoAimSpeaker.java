package frc.robot.commands;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.LimeLight;
import frc.robot.subsystems.Shooter;
import frc.robot.lib.Constants.ShooterSubsystemConstants;
import frc.robot.lib.Constants.FieldGeometry.Speaker.ShootingPosition;

public class autoAimSpeaker extends Command {

    public LimeLight m_LimeLight;
    public Shooter m_Shooter;
    
    public autoAimSpeaker(Shooter _shooter, LimeLight _LimeLight) {
        addRequirements(_shooter,_LimeLight);
        m_LimeLight = _LimeLight;
        m_Shooter = _shooter;
    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double[] positionData = m_LimeLight.targetpose_cameraspace();
        double roboDistance = Units.metersToInches(positionData[2]) - ShootingPosition.shootingPosDistanceOffset;
        double desiredAngle = Units.radiansToDegrees(Math.atan( (ShootingPosition.shootingPosHeight - ShooterSubsystemConstants.SHOOTER_HEIGHT ) / roboDistance));

        SmartDashboard.putNumber("Limelight Target Angle", desiredAngle);

        if (desiredAngle > 0.0 && desiredAngle < 50.0) {
            m_Shooter.setTilter(desiredAngle);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_Shooter.setTilter(60.0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }


    
}
