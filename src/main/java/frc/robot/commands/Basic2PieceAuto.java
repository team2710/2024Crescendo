package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class Basic2PieceAuto extends SequentialCommandGroup {
    
    public Basic2PieceAuto(DriveSubsystem robotDrive) {
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("2 Note Auto");
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile("2 Note Auto");
        // robotDrive.resetOdometry(startingPose);

        addCommands(
            Commands.runOnce(robotDrive::zeroHeading),
            Commands.runOnce(() -> robotDrive.getGyro().setAngleAdjustment(startingPose.getRotation().getDegrees())),
            Commands.runOnce(() -> robotDrive.resetOdometryWithAlliance(startingPose)),
            AutoBuilder.followPath(pathGroup.get(0))
        );
    }

}
