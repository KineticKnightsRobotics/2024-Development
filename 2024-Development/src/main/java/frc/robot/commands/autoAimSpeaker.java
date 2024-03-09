package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.lib.Constants.FieldGeometry.Speaker.ShootingPosition;

public class autoAimSpeaker extends Command {

    public Shooter m_Shooter;
    public SwerveDrive m_Drive;

    public double angle, distance;
    
    public autoAimSpeaker(Shooter _shooter, SwerveDrive _drive) {
        addRequirements(_shooter);
        m_Shooter = _shooter;
        m_Drive = _drive;
    }
    @Override
    public void initialize() {



        /* 
        double[] positionData = LimelightHelpers.getTargetPose_RobotSpace("limelight");
        double roboDistance = Units.metersToInches(positionData[2]) - ShootingPosition.shootingPosDistanceOffset - 14; //14 = half robot length
        //double desiredAngle = Units.radiansToDegrees(Math.atan( (ShootingPosition.shootingPosHeight - ShooterSubsystemConstants.SHOOTER_HEIGHT ) / roboDistance));
        SmartDashboard.putNumber("Limelight Target Angle", m_Shooter.shooterInterpolator.interpolateAngle(roboDistance));
        m_Shooter.setTilter(m_Shooter.shooterInterpolator.interpolateAngle(roboDistance));
        */
    }

    @Override
    public void execute() {



        distance = m_Drive.getDistanceToSpeaker() - Units.inchesToMeters(39) - Units.inchesToMeters(14); //TODO: remove the math once the shooter interpolator remeasured.

        angle = m_Shooter.shooterInterpolator.interpolateAngle(distance);

        m_Shooter.setShooterPosition(angle);


        //m_Shooter.setTilter(m_Shooter.shooterInterpolator.interpolateAngle(m_Drive.getTranslationRelativeToSpeaker()));
    }
    @Override
    public void end(boolean interrupted) {
        //m_Shooter.setTilter(0.0);
    }
    @Override
    public boolean isFinished() {
        return (m_Shooter.getTilterPosition() == angle);
    }


    
}
