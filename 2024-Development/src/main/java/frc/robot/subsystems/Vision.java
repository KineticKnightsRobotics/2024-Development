package frc.robot.subsystems;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.LimelightHelpers;
import frc.robot.lib.Constants.VisionConstants.defaultSTD;

public class Vision {
    
    SwerveDrive m_Drive;
    public Vision(SwerveDrive _Drive){
        m_Drive = _Drive;
    }

    public Pose2d getEstimatedRoboPose() {
        return LimelightHelpers.getBotPose2d_wpiBlue("limelight");
    }

    public Matrix<N3,N1> getStandardDeviations(){
        LimelightHelpers.PoseEstimate limelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        //Get tag count and average position...
        int tagCount = limelightPose.tagCount;
        double avgDist = limelightPose.avgTagDist;

        if ( (tagCount == 1 && avgDist > 4) || (m_Drive.ODEMETER.getEstimatedPosition().getTranslation().getDistance(limelightPose.pose.getTranslation()) > 0.5) ) {
            return VecBuilder.fill(Double.MAX_VALUE,Double.MAX_VALUE,Double.MAX_VALUE);
        }
        else {
            return defaultSTD.singleTagStD.times(1 + (Math.pow(avgDist, 2) / 30)); //Borrowed equation from 3161 :]
        }
    }


}
