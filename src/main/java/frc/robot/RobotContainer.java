// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.DriverProfile.mathProfiles;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.Basic2PieceAuto;
import frc.robot.commands.ShootIntake;
import frc.robot.commands.autoRotatePP;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Pivot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();


  // The driver's controller
  CommandPS4Controller m_driverControllerCommand = new CommandPS4Controller(OIConstants.kDriverControllerPort);
  PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort);
  CommandPS4Controller m_auxController = new CommandPS4Controller(OIConstants.kAuxControllerPort);
  Climb climber = new Climb();
  LEDSubsystem ledSubsystem = new LEDSubsystem();
  EndEffector endEffector = new EndEffector(m_driverController, ledSubsystem, climber);
  Pivot pivot = new Pivot(m_robotDrive, ledSubsystem);

  Command spinupFlywheelCommand = new InstantCommand(() -> endEffector.spinupFlywheel(), endEffector);
  Command stopFlywheelCommand = new InstantCommand(() -> endEffector.stopFlywheel(), endEffector);
  Command autoIntake = new AutoIntake(endEffector);

  // Pivot Arm


  Command PathfindToPickUp = Commands.none();
  Command PathfindToScore = Commands.none();


  Command PathfindToPickupBlue = AutoBuilder.pathfindToPose(
    new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
    new PathConstraints(
      4.0, 4.0, 
      Units.degreesToRadians(360), Units.degreesToRadians(540)
    ), 
    0, 
    2.0
  );

  Command PathfindToPickupRed = 
  AutoBuilder.pathfindToPose(
    new Pose2d(0.956, 1.763, Rotation2d.fromDegrees(180)), 
    new PathConstraints(
      4.0, 4.0, 
      Units.degreesToRadians(360), Units.degreesToRadians(540)
    ), 
    0, 
    2.0
  );

  Command PathfindToScoringBlue = AutoBuilder.pathfindToPose(
    new Pose2d(2.15, 5.5, Rotation2d.fromDegrees(180)), 
    new PathConstraints(
      4.0, 4.0, 
      Units.degreesToRadians(0), Units.degreesToRadians(540)
    ), 
    0, 
    0
  );

  Command PathfindToScoringRed = AutoBuilder.pathfindToPose(
    new Pose2d(13.0, 5.5, Rotation2d.fromDegrees(0)), 
    new PathConstraints(
      4.0, 4.0, 
      Units.degreesToRadians(0), Units.degreesToRadians(540)
    ), 
    0, 
    0
  );

  // Command pivotSpeaker = new RunCommand(() -> pivot.PivotStateSetter(Pivot.PivotState.Speaker), pivot);
  // Command pivotAMP = new RunCommand(() -> pivot.PivotStateSetter(Pivot.PivotState.AMP), pivot);
  // Command pivotOff = new RunCommand(() -> pivot.PivotStateSetter(Pivot.PivotState.OFF), pivot);


  final Trigger driverL1 = m_driverControllerCommand.L1();
  final Trigger driverL2 = m_driverControllerCommand.L2();
  final Trigger driverR1 = m_driverControllerCommand.R1();
  final Trigger driverR2 = m_driverControllerCommand.R2();
  final Trigger driverCross = m_driverControllerCommand.cross();
  final Trigger driverCircle = m_driverControllerCommand.circle();
  final Trigger driverTriangle = m_driverControllerCommand.triangle();
  final Trigger driverDPADUP = m_driverControllerCommand.povUp();
  final Trigger driverDPADDOWN = m_driverControllerCommand.povDown();
  final Trigger driverSquare = m_driverControllerCommand.square();


  final Trigger auxL1 = m_auxController.L1();
  final Trigger auxR1 = m_auxController.R1();
  final Trigger auxCross = m_auxController.cross();
  final Trigger auxCircle = m_auxController.circle();
  final Trigger auxTriangle = m_auxController.triangle();
  final Trigger auxR2 = m_auxController.R2();
  final Trigger auxL2 = m_auxController.L2();
  final Trigger auxDPADUP = m_auxController.povUp();
  final Trigger auxDPADDOWN = m_auxController.povDown();

  final Trigger auxSquare = m_auxController.square();

  boolean isRed = false;

  HttpCamera camera = new HttpCamera("Limelight", "http://10.27.10.11:5800/stream.mjpg");

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  // AUTO COMMANDS

  Command shootCommand = Commands.sequence(
    endEffector.stopIntakeCommand(),
    endEffector.toggleOuttakeCommand(),
    Commands.waitSeconds(0.11),
    endEffector.stopIntakeCommand(),
    endEffector.flywheelAutoCommand(),
    Commands.race(
      new WaitUntilCommand(endEffector::atShootingSpeed),
      Commands.waitSeconds(3)
    ),
    endEffector.feedCommand(),
    Commands.waitSeconds(0.2),
    endEffector.stopIntakeCommand(),
    endEffector.stopFlywheelCommand()
  );

  // Command shootWithoutFlyWheel = Commands.parallel(
  //   endEffector.stopIntakeCommand(),  
  //   endEffector.toggleOuttakeCommand(),
  //   Commands.waitSeconds(0.12),
  //   endEffector.stopIntakeCommand(),
  //   endEffector.feedCommand(),
  //   Commands.waitSeconds(0.15),
  //   endEffector.stopIntakeCommand(),
  //   endEffector.stopFlywheelCommand()
  // );



  // Command shootCommand = Commands.sequence(
  //   autoFeedAndShoot,
  //   endEffector.stopIntakeCommand(),
  //   endEffector.stopFlywheelCommand()

  // );

  Command lowerArmCommand = Commands.sequence(
    pivot.pivotMoveCommand(0),
    new WaitUntilCommand(pivot::reachedSetpoint)
  );

  Command autoIntakeSeq = Commands.sequence(
    // lowerArmCommand,
    // endEffector.toggleIntakeCommand()
    autoIntake,
    new WaitCommand(3),
    endEffector.stopIntakeCommand()
  );

  Command autoShoot = Commands.sequence(
    pivot.autoAimPivotCommand(),
    new WaitUntilCommand(pivot::reachedSetpoint),
    shootCommand,
    lowerArmCommand
  );

  // Command autoShoot2 = new SequentialCommandGroup(
  //   new ParallelCommandGroup(
  //     new SequentialCommandGroup(pivot.autoAimPivotCommand(), new WaitUntilCommand(pivot::reachedSetpoint)),
  //     shootCommand
  //   ),
  //   lowerArmCommand
  // );


  
  
    Command outtakeCommand = Commands.sequence(
      endEffector.outtakeCommand(),
      Commands.waitSeconds(0.12),
      endEffector.stopIntakeCommand()
    );
  
  

  // private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
       isRed = (alliance.get() == DriverStation.Alliance.Red);
    }
    if(isRed == false) {
      PathfindToPickUp = PathfindToPickupBlue;
      PathfindToScore  = PathfindToScoringBlue;
   
      // driverDPADDOWN.whileTrue(PathfindToScoringBlue);
      // driverDPADUP.whileTrue(PathfindToPickupBlue);
    } else {
      PathfindToPickUp = PathfindToPickupRed;
      PathfindToScore  = PathfindToScoringRed;
      // driverDPADDOWN.whileTrue(PathfindToScoringRed);
      // driverDPADUP.whileTrue(PathfindToPickupRed);
    }


    Command PathFindtoScoreSeq = Commands.sequence(
      m_robotDrive.velocityControlEnabledCommand(true),
      PathfindToScore,
      m_robotDrive.velocityControlEnabledCommand(false),
      autoShoot
    );

    Command PathFindtoPickupSeq = Commands.sequence(
      m_robotDrive.velocityControlEnabledCommand(true),
      PathfindToPickUp,
      m_robotDrive.velocityControlEnabledCommand(false)
    );

    SmartDashboard.putData("PathFindToScore", PathFindtoScoreSeq);
    SmartDashboard.putData("PathFindToPickUp", PathFindtoPickupSeq);


    // Commands for Pathplanner
    NamedCommands.registerCommand("pivot_subwoofer", lowerArmCommand);
    NamedCommands.registerCommand("shoot", shootCommand);
    NamedCommands.registerCommand("outtake", outtakeCommand);
    NamedCommands.registerCommand("auto_shoot", autoShoot);
    // NamedCommands.registerCommand("auto_intake_sequence", Commands.race(new AutoIntake(endEffector), Commands.waitSeconds(3
    // )));
    NamedCommands.registerCommand("auto_intake_sequence", 
      Commands.sequence(
        endEffector.intakeCommand(),
        Commands.waitSeconds(2),
        endEffector.stopIntakeCommand()
      )
    );
    NamedCommands.registerCommand("toggle_intake", endEffector.toggleIntakeCommand());
    NamedCommands.registerCommand("outtake", outtakeCommand);

    // Autos
    autoChooser.setDefaultOption("No Auto", Commands.none());
    autoChooser.addOption("Basic 2 Piece", new PathPlannerAuto("2 Note Auto"));
    autoChooser.addOption("2 Piece top", new PathPlannerAuto("2 Note Subwoofer Top"));
    autoChooser.addOption("6 Note", new PathPlannerAuto("Note 6 Normal"));
    autoChooser.addOption("5 Note Far Left", new PathPlannerAuto("5 Note Far Left"));
    autoChooser.addOption("5 Note Right", new PathPlannerAuto("Note 5"));
    autoChooser.addOption("6 Note through Stage", new PathPlannerAuto("Note 6 Stage"));
    autoChooser.addOption("1 Piece Auto", Commands.sequence(
      pivot.pivotMoveCommand(0),
      Commands.waitSeconds(2),
      endEffector.toggleFlywheelCommand(),
      Commands.waitSeconds(1),
      endEffector.toggleIntakeCommand(),
      Commands.waitSeconds(0.5),
      endEffector.toggleIntakeCommand(),
      endEffector.toggleFlywheelCommand()
    ));
    autoChooser.addOption("3 Piece", new PathPlannerAuto("3 Note Auto"));
    autoChooser.addOption("4 Piece", new PathPlannerAuto("4 Note Auto"));
    autoChooser.addOption("4 Piece Real", new PathPlannerAuto("4 Note Auto Real"));
    autoChooser.addOption("Two Piece Bottom to White Centerlin", new PathPlannerAuto("Bottom to White Centerline Auto"));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // kbShooter = new KitBotShooter();

    // Not working
    CameraServer.addCamera(camera);
    Shuffleboard.getTab("Limelight").add(camera);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
           new RunCommand(
            () -> m_robotDrive.DriverDrive(
                -mathProfiles.exponentialDrive(MathUtil.applyDeadband(m_driverControllerCommand.getLeftY(), OIConstants.kDriveDeadband), 2),
                -mathProfiles.exponentialDrive(MathUtil.applyDeadband(m_driverControllerCommand.getLeftX(), OIConstants.kDriveDeadband), 2),
                -MathUtil.applyDeadband(m_driverControllerCommand.getRightX(), OIConstants.kDriveDeadband),
                true, true, m_driverController, false, false),
            m_robotDrive));

    m_robotDrive.zeroHeading();
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  /**
   * Configures the button bindings for the robot.
   * Binds various commands to different buttons on the controller.
   */
  /**
   * Configures the button bindings for the robot.
   * Associates each button with its corresponding command or action.
   */
  private void configureButtonBindings() {
    // AUX COMMANDS

    // END EFFECTOR COMMANDS

    auxR1.onTrue(endEffector.toggleFlywheelCommand());

    // auxTriangle.onTrue(new InstantCommand(() -> {
    //   endEffector.intake();
    // })).onFalse(new InstantCommand(() -> {
    //   endEffector.stopIntake();
    // }));
    auxTriangle.onTrue(new AutoIntake(endEffector)).onFalse(new InstantCommand(() -> {
      endEffector.stopIntake();
    }, endEffector));
    auxSquare.onTrue(new InstantCommand(() -> {
      endEffector.outtake();
    })).onFalse(new InstantCommand(() -> {
      endEffector.stopIntake();
    }));
    
    // AUTO COMMANDS
    auxCircle.whileTrue(new RunCommand(() -> {
      pivot.autoAimPivot();
    })).onFalse(new InstantCommand(() -> {
      pivot.setAngleDegree(0);
    }));

    // CLIMB COMMANDS
    auxDPADUP.onTrue(new InstantCommand(() -> {
      climber.climbUP();
    })).onFalse( new InstantCommand(() -> {
      climber.stopClimb();
    }));

    auxDPADDOWN.onTrue(new InstantCommand(() -> {
      climber.climbDown();
    })).onFalse( new InstantCommand(() -> {
      climber.stopClimb();
    }));

    auxL2.onTrue(new InstantCommand(() -> {
      endEffector.ampFlywheel();
    }, endEffector)).onFalse(new InstantCommand(() -> {
      endEffector.stopFlywheel();
      endEffector.stopIntake();
    }));

    // PIVOT COMMANDS
    // auxL2.onTrue(pivot.pivotMoveCommand(PivotConstants.kPivotAmpAngle));
    // auxCross.onTrue(pivot.pivotMoveCommand(PivotConstants.kPivotZero));

    // DRIVER COMMANDS
    driverCross.onTrue(new InstantCommand(() -> {
      m_robotDrive.zeroHeading();
    }, m_robotDrive));

    driverCircle.onTrue(new InstantCommand(() -> {
      pivot.zeroPivot();
    }, pivot));

    // Right Bumper is Zero
    driverR1.onTrue(pivot.pivotMoveCommand(PivotConstants.kPivotZero));
    driverR2.onTrue(endEffector.feedCommand()).onFalse(endEffector.stopIntakeCommand());
    driverL2.onTrue(pivot.pivotMoveCommand(PivotConstants.kPivotStow));
    driverL1.onTrue(pivot.pivotMoveCommand(PivotConstants.kPivotAmpAngle));
    driverSquare.whileTrue(new RunCommand(() -> {
      pivot.autoAimPivot();
    }, pivot)).onFalse(pivot.pivotMoveCommand(PivotConstants.kPivotZero));

    // Shouldnt be used
    driverTriangle.onTrue(new InstantCommand(() -> {
      pivot.disable();
    }, pivot)).onFalse(new InstantCommand(() -> {
      pivot.enable();
    }, pivot));

    // SHOOOOOT
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
