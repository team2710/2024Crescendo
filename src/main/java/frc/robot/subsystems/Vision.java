package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {

    private String limelightName = "limelight";

    public Vision(String limelightName) {
        this.limelightName = limelightName;
    }

    public int getTagID() {
        return (int)LimelightHelpers.getFiducialID(limelightName);
    }

    public boolean hasValidTargets() {
        return LimelightHelpers.getTV(limelightName);
    }

    public Pose2d getBotPose2d() {
        return LimelightHelpers.getBotPose2d(limelightName);
    }

    public Pose3d getPosePose3d() {
        return LimelightHelpers.getBotPose3d(limelightName);
    }

    public double getLatency() {
        double tl = LimelightHelpers.getLatency_Pipeline(limelightName);
        double cl = LimelightHelpers.getLatency_Capture(limelightName);
        return (tl/1000.0) + (cl/1000.0);
    }

    @Override
    public void periodic() {
        boolean validTargets = hasValidTargets();

        SmartDashboard.putBoolean("Has Vision Targets", validTargets);
        if (validTargets) {
            SmartDashboard.putNumber("Fiducial ID", getTagID());
        }
    }

}
