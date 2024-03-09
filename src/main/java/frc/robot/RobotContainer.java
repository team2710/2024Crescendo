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
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.Basic2PieceAuto;
import frc.robot.commands.autoRotatePP;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.EndEffector.FlywheelState;
import frc.robot.subsystems.EndEffector.IntakeState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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
  public static final EndEffector endEffector = new EndEffector();
    Pivot pivot = new Pivot(m_robotDrive);


  // The driver's controller
  CommandPS4Controller m_driverControllerCommand = new CommandPS4Controller(OIConstants.kDriverControllerPort);
  PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort);
  CommandPS4Controller m_auxController = new CommandPS4Controller(OIConstants.kAuxControllerPort);

  // KitBotShooter kbShooter = new KitBotShooter();
  // Commands
  Command intakeCommand = new InstantCommand(() -> endEffector.setIntakeState(IntakeState.Feed), endEffector);
  Command stopIntake = new InstantCommand(() -> endEffector.setIntakeState(IntakeState.OFF), endEffector);
  Command intakeOn = new InstantCommand(() -> endEffector.setIntakeState(IntakeState.On), endEffector);
  Command outtake = new InstantCommand(() -> endEffector.setIntakeState(IntakeState.Outtake));

  Command pivotZero = new InstantCommand(() -> pivot.zeroPivot(), pivot);


  Command spinupFlywheelCommand = new InstantCommand(() -> endEffector.spinupFlywheel(), endEffector);
  Command stopFlywheelCommand = new InstantCommand(() -> endEffector.stopFlywheel(), endEffector);

  // Pivot Arm
  Climb climber = new Climb();

 Command PathfindToPickupBlue = AutoBuilder.pathfindToPose(
    new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
    new PathConstraints(
      4.0, 4.0, 
      Units.degreesToRadians(360), Units.degreesToRadians(540)
    ), 
    0, 
    2.0
  );

  Command PathfindToPickupRed = AutoBuilder.pathfindToPose(
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
      Units.degreesToRadians(360), Units.degreesToRadians(540)
    ), 
    0, 
    0
  );

  Command PathfindToScoringRed = AutoBuilder.pathfindToPose(
    new Pose2d(13.0, 5.5, Rotation2d.fromDegrees(0)), 
    new PathConstraints(
      4.0, 4.0, 
      Units.degreesToRadians(360), Units.degreesToRadians(540)
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
  final Trigger driverDPADUP = m_driverControllerCommand.povUp();
  final Trigger driverDPADDOWN = m_driverControllerCommand.povDown();


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
    endEffector.toggleFlywheelCommand(),
    Commands.waitSeconds(0.5),
    endEffector.toggleIntakeCommand(),
    Commands.waitSeconds(0.5),
    endEffector.toggleIntakeCommand(),
    endEffector.toggleFlywheelCommand()
  );

    Command shootOnMoveCommand = Commands.race(
    new autoRotatePP(true, m_robotDrive),
    pivot.autoAimPivotCommand().withTimeout(1.1),
    shootCommand
  );

  Command lowerArmCommand = Commands.sequence(
    pivot.pivotMoveCommand(0),
    Commands.waitSeconds(0.5)
  );

  Command intakeToggleSeq = Commands.sequence(
    new autoRotatePP(false, m_robotDrive),
    pivotZero,
    endEffector.toggleIntakeCommand()
  );
  

  // private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Commands for Pathplanner
    NamedCommands.registerCommand("pivot_subwoofer", lowerArmCommand);
    NamedCommands.registerCommand("shoot", shootCommand);
    NamedCommands.registerCommand("toggle_intake", endEffector.toggleIntakeCommand());
    NamedCommands.registerCommand("auto_aim", shootOnMoveCommand);
    NamedCommands.registerCommand("toggle_intake", intakeToggleSeq);



    autoChooser.setDefaultOption("No Auto", Commands.none());
    autoChooser.addOption("Basic 2 Piece", new PathPlannerAuto("2 Note Auto"));
    autoChooser.addOption("2 Piece top", new PathPlannerAuto("2 Note Subwoofer Top"));
    autoChooser.addOption("8 Note", new PathPlannerAuto("8 Note Auto"));


    autoChooser.addOption("1 Piece Auto", Commands.sequence(
      pivot.pivotMoveCommand(0),
      Commands.waitSeconds(3),
      endEffector.toggleFlywheelCommand(),
      Commands.waitSeconds(1),
      endEffector.toggleIntakeCommand(),
      Commands.waitSeconds(0.5),
      endEffector.toggleIntakeCommand(),
      endEffector.toggleFlywheelCommand()
    ));
    autoChooser.addOption("3 Piece", new PathPlannerAuto("3 Note Auto"));
    autoChooser.addOption("4 Piece", new PathPlannerAuto("4 Note Auto"));
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // kbShooter = new KitBotShooter();

    CameraServer.addCamera(camera);
    Shuffleboard.getTab("Limelight").add(camera);

    // Configure the button bindings
    configureButtonBindings();

    // autoChooser = AutoBuilder.buildAutoChooser("3 Note");
    // SmartDashboard.putData(autoChooser);
    // If you wanna test FSM uncomment
    // endEffector.setDefaultCommand(new RunCommand(() ->
    // endEffector.IntakeSetter(m_driverController), endEffector));

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.

        //square button for aim assist
        //uncomment if no worky
        new RunCommand(
            () -> m_robotDrive.DriverDrive(
                -MathUtil.applyDeadband(m_driverControllerCommand.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverControllerCommand.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverControllerCommand.getRightX(), OIConstants.kDriveDeadband),
                true, true, m_driverController),
            m_robotDrive));

          //this is old
          // new RunCommand(
          //   () -> m_robotDrive.drive(
          //       -MathUtil.applyDeadband(m_driverControllerCommand.getLeftY(), OIConstants.kDriveDeadband),
          //       -MathUtil.applyDeadband(m_driverControllerCommand.getLeftX(), OIConstants.kDriveDeadband),
          //       -MathUtil.applyDeadband(m_driverControllerCommand.getRightX(), OIConstants.kDriveDeadband),
          //       true, false),
          //   m_robotDrive));

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
    auxTriangle.onTrue(new InstantCommand(() -> {
      endEffector.intake();
    })).onFalse(new InstantCommand(() -> {
      endEffector.stopIntake();
    }));
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

    // PIVOT COMMANDS
    auxL2.onTrue(pivot.pivotMoveCommand(PivotConstants.kPivotAmpAngle));
    auxL1.onTrue(pivot.pivotMoveCommand(PivotConstants.kPivotPodiumAngle));
    auxR2.onTrue(pivot.pivotMoveCommand(PivotConstants.kPivotWingAngle));
    auxCross.onTrue(pivot.pivotMoveCommand(PivotConstants.kPivotZero));
    driverL2.onTrue(pivot.pivotMoveCommand(PivotConstants.kPivotStow));

    // var alliance = DriverStation.getAlliance();
    // if (alliance.isPresent()) {
    //    isRed = (alliance.get() == DriverStation.Alliance.Red);
    // }
    // if(isRed == false) {
    //   SmartDashboard.putData("pickup",PathfindToPickupBlue);
    //   SmartDashboard.putData("scoring",PathfindToScoringBlue);
    //   driverDPADDOWN.whileTrue(PathfindToScoringBlue);
    //   driverDPADUP.whileTrue(PathfindToPickupBlue);
    // } else {
    //   SmartDashboard.putData("pickup",PathfindToPickupRed);
    //   SmartDashboard.putData("scoring",PathfindToScoringRed);
    //   driverDPADDOWN.whileTrue(PathfindToScoringRed);
    //   driverDPADUP.whileTrue(PathfindToPickupRed);
    // }



    // trigger and state machine (prob better implemenetation)
    // uncomment to test
    // auxR1.onTrue(shooterOn).onFalse(shooterOff);
    // auxTriangle.onTrue(intakeOn).onFalse(stopIntake);
    // auxL1.onTrue(feed).onFalse(stopIntake);
    // auxSquare.onTrue(outtake).onFalse(stopIntake);

    
    // auxR2.onTrue(new InstantCommand(() -> {
    //   pivot.setAngleDegree(0);
    // }));
    // auxL2.onTrue(new InstantCommand(() -> {
    //   pivot.setAngleDegree(80);
    // }));

    // auxCross.onTrue(new InstantCommand(() -> {
    //   pivot.setAngleDegree(80);
    // }));

    // DRIVER COMMANDS
    driverCross.onTrue(new InstantCommand(() -> {
      m_robotDrive.zeroHeading();
    }, m_robotDrive));

    driverCircle.onTrue(new InstantCommand(() -> {
      pivot.zeroPivot();
    }, pivot));

    driverL1.onTrue(new InstantCommand(() -> {
      pivot.disable();
    }, pivot)).onFalse(new InstantCommand(() -> {
      pivot.enable();
    }, pivot));

    driverR2.onTrue(new InstantCommand(() -> {
      endEffector.intake();
    })).onFalse(new InstantCommand(() -> {
      endEffector.stopIntake();
    }));

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
