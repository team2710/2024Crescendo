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

public class IntakeWithBB extends Command {
    EndEffector m_intake;

    public IntakeWithBB(EndEffector m_intake) {
        this.m_intake = m_intake;
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here
        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intake.intake();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.stopIntake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_intake.isNoteDetected();
    }

}
