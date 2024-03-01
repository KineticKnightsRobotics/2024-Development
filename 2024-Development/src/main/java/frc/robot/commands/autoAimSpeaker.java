package frc.robot.commands;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Shooter;
import frc.robot.lib.Constants.FieldGeometry.Speaker.ShootingPosition;

public class autoAimSpeaker extends Command {

    public Shooter m_Shooter;
    
    public autoAimSpeaker(Shooter _shooter) {
        addRequirements(_shooter);
        m_Shooter = _shooter;
    }
    @Override
    public void initialize() {
        double[] positionData = LimelightHelpers.getTargetPose_RobotSpace("limelight");
        double roboDistance = Units.metersToInches(positionData[2]) - ShootingPosition.shootingPosDistanceOffset - 14; //14 = half robot length
        //double desiredAngle = Units.radiansToDegrees(Math.atan( (ShootingPosition.shootingPosHeight - ShooterSubsystemConstants.SHOOTER_HEIGHT ) / roboDistance));
        SmartDashboard.putNumber("Limelight Target Angle", m_Shooter.shooterInterpolator.interpolateAngle(roboDistance));
        m_Shooter.setTilter(m_Shooter.shooterInterpolator.interpolateAngle(roboDistance));
    }
    @Override
    public void end(boolean interrupted) {
        //m_Shooter.setTilter(0.0);
    }
    @Override
    public boolean isFinished() {
        return true;
    }


    
}
