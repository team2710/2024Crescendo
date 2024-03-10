package frc.robot.commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Pivot;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;

public class StopFlywheelAndIntake extends Command {
    EndEffector m_end;

    public StopFlywheelAndIntake(EndEffector m_end) {
        this.m_end = m_end;
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here
        addRequirements(m_end);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_end.stopFlywheel();
        m_end.stopIntake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){      
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

}
