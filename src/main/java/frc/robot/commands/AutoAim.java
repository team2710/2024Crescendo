package frc.robot.commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Pivot;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;

public class AutoAim extends Command {
        Pose2d poseRedSpeaker = new Pose2d(0,2.655,Rotation2d.fromDegrees(0));
        Pose2d poseBlueSpeaker = new Pose2d(0,2.655,Rotation2d.fromDegrees(0));
        

        public AutoAim(Pivot m_pivot, DriveSubsystem m_drive) {
            // Use addRequirements() here to declare subsystem dependencies.
            // Configure additional PID options by calling `getController` here
            addRequirements(null);
        }
        
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
        }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
