package frc.robot.commands;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Shooter;
import frc.robot.lib.Constants.ShooterSubsystemConstants;
import frc.robot.lib.Constants.FieldGeometry.Speaker.ShootingPosition;

public class autoAimSpeaker extends Command {

    public Shooter m_Shooter;
    
    public autoAimSpeaker(Shooter _shooter) {
        addRequirements(_shooter);
        m_Shooter = _shooter;
    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double[] positionData = LimelightHelpers.getTargetPose_RobotSpace("limelight");
        double roboDistance = Units.metersToInches(positionData[2]) - ShootingPosition.shootingPosDistanceOffset - 14; //14 = half robot length
        double desiredAngle = Units.radiansToDegrees(Math.atan( (ShootingPosition.shootingPosHeight - ShooterSubsystemConstants.SHOOTER_HEIGHT ) / roboDistance));

        SmartDashboard.putNumber("Limelight Target Angle", desiredAngle);

        if (desiredAngle > 0.0 && desiredAngle < 50.0) {
            m_Shooter.setTilterPosition(60 - desiredAngle);
        }


    }

    @Override
    public void end(boolean interrupted) {
        m_Shooter.setTilter(0.0);
    }
    @Override
    public boolean isFinished() {
        return true;
    }


    
}
