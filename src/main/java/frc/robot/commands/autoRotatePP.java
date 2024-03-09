package frc.robot.commands;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class autoRotatePP extends Command {
    DriveSubsystem swerve;
    Optional<Rotation2d> angle = Optional.empty();
    boolean override = false;
    
    
    public autoRotatePP(boolean override, DriveSubsystem swerve) {
        this.swerve = swerve;
        this.override = override;
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    public Optional<Rotation2d> getAngle() {
        return angle;
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        angle = swerve.autoAimPP(override);
        // SmartDashboard.putNumber("Angle", angle.get().getDegrees());
        PPHolonomicDriveController.setRotationTargetOverride(this::getAngle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if(!override){
        PPHolonomicDriveController.setRotationTargetOverride(Optional::empty);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    
}
