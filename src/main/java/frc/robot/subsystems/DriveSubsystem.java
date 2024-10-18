// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.google.flatbuffers.Constants;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.LimelightHelpers;
import frc.utils.SwerveUtils;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // The vision subsystem
  private final Vision vision = new Vision("limelight");

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;
  boolean isRed = false;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  public Translation3d target_pose = DriveConstants.blueSpeaker;

  public PIDController m_botAnglePID = new PIDController(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD);

  LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

  // Instead of using odometry class, we use a pose esimator to allow fusing multiple inputs
  SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      limelightMeasurement.pose);

  private Field2d m_field = new Field2d();
  RobotConfig config;

  

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SmartDashboard.putData("Field", m_field);
    zeroHeading();

    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  // Configure AutoBuilder last
  AutoBuilder.configure(
    this::getPose, // Robot pose supplier
    this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
            new com.pathplanner.lib.config.PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new com.pathplanner.lib.config.PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    ),
    config, // The robot configuration
    () -> {
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    },
    this // Reference to this subsystem to set requirements
);
}


  // PATHPLANNER STUFF
  public void enableVelocityControl(boolean control){
    m_frontLeft.velocityControlEnabled(control);
    m_frontRight.velocityControlEnabled(control);
    m_rearLeft.velocityControlEnabled(control);
    m_rearRight.velocityControlEnabled(control);
  }

  ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

    ChassisSpeeds getRobotDiscreteSpeeds() {
    return ChassisSpeeds.discretize(getRobotRelativeSpeeds(), AutoConstants.kSwerveDiscreteTimestep);
  }

  void driveRobotRelative(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, AutoConstants.kSwerveDiscreteTimestep);
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }

  SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };
  }

  // END PATHPLANNER STUFF

  @Override
  public void periodic() {
    SmartDashboard.putNumber("vx", getRobotRelativeSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("vy", getRobotRelativeSpeeds().vyMetersPerSecond);


    SmartDashboard.putNumber("Gyro", getHeading());

    // Advantage Kit Swerve Logging
    Logger.recordOutput("ModuleStates", getModuleStates());

    // Update the odometry in the periodic block
    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    
    //uncomment for bruh
    limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    SmartDashboard.putNumber("Limelight tag count", limelightMeasurement.tagCount);
    if(vision.hasValidTargets())
    {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      m_poseEstimator.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds);
    }

    m_field.setRobotPose(getPose());
    Logger.recordOutput("botpose_blue", limelightMeasurement.pose);
    Logger.recordOutput("position", m_field.getRobotPose());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public Optional<Rotation2d> overridePPRotation(){
    double angle = autoAim(getRobotRelativeSpeeds().vxMetersPerSecond, getRobotRelativeSpeeds().vyMetersPerSecond, getPose(), 45, target_pose.toTranslation2d());
    return Optional.of(Rotation2d.fromDegrees(angle));    
  }

  public Command overridePPRotationCommand(){
    return new InstantCommand(() -> {
      PPHolonomicDriveController.setRotationTargetOverride(this::overridePPRotation);
    }, this);
  }

  public AHRS getGyro() {
    return m_gyro;
  }

  public Optional<Rotation2d> autoAimPP(boolean override){
    double angle = autoAim(getRobotRelativeSpeeds().vxMetersPerSecond, getRobotRelativeSpeeds().vyMetersPerSecond, getPose(), 45, target_pose.toTranslation2d());
    if(override){
      return Optional.of(Rotation2d.fromRadians(angle));
    }
    return Optional.empty(); 
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    // double angle = pose.getRotation().getDegrees();
    // m_gyro.setAngleAdjustment(angle);
    m_poseEstimator.resetPosition(getGyro().getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  public double autoAim(double vx, double vy, Pose2d robotPose, double shooterVelocity, Translation2d targetPose) {
    // Extract robot's current position
    double rx = robotPose.getX();
    double ry = robotPose.getY();

    // Extract target's position
    double tx = targetPose.getX();
    double ty = targetPose.getY();

    // Calculate relative position (dx, dy) from robot to target
    double dx = tx - rx;
    double dy = ty - ry;

    // Debug: Display relative position
    SmartDashboard.putNumber("Relative Position X (dx)", dx);
    SmartDashboard.putNumber("Relative Position Y (dy)", dy);

    // Calculate the coefficients of the quadratic equation: a*t^2 + b*t + c = 0
    // Derived from the equation: (dx + vx * t)^2 + (dy + vy * t)^2 = (v_p * t)^2
    double a = (vx * vx + vy * vy - shooterVelocity * shooterVelocity);
    double b = 2 * (dx * vx + dy * vy);
    double c = dx * dx + dy * dy;

    // Debug: Display quadratic coefficients
    SmartDashboard.putNumber("Quadratic Coefficient a", a);
    SmartDashboard.putNumber("Quadratic Coefficient b", b);
    SmartDashboard.putNumber("Quadratic Coefficient c", c);

    // Calculate discriminant
    double discriminant = b * b - 4 * a * c;

    // Debug: Display discriminant
    SmartDashboard.putNumber("Discriminant", discriminant);

    // Check if the discriminant is negative or a is zero (no solution)
    if (discriminant < 0 || Math.abs(a) < 1e-6) {
        SmartDashboard.putString("AutoAim Status", "No solution: Target cannot be hit with the given projectile speed.");
        throw new IllegalArgumentException("No solution: Target cannot be hit with the given projectile speed.");
    }

    // Calculate both possible times to impact
    double sqrtDiscriminant = Math.sqrt(discriminant);
    double t1 = (-b + sqrtDiscriminant) / (2 * a);
    double t2 = (-b - sqrtDiscriminant) / (2 * a);

    // Debug: Display possible times to impact
    SmartDashboard.putNumber("Time to Impact t1", t1);
    SmartDashboard.putNumber("Time to Impact t2", t2);

    // Select the smallest positive time
    double tImpact = Double.MAX_VALUE;
    if (t1 > 0 && t1 < tImpact) {
        tImpact = t1;
    }
    if (t2 > 0 && t2 < tImpact) {
        tImpact = t2;
    }

    // Check if a positive time solution exists
    if (tImpact == Double.MAX_VALUE) {
        SmartDashboard.putString("AutoAim Status", "No positive time solution: Target cannot be hit.");
        throw new IllegalArgumentException("No positive time solution: Target cannot be hit.");
    }

    // Debug: Display selected time to impact
    SmartDashboard.putNumber("Selected Time to Impact (s)", tImpact);

    // Calculate the required projectile velocity components to hit the target in tImpact seconds
    double Vpx = dx / tImpact;
    double Vpy = dy / tImpact;

    // Adjust for the robot's velocity to get the projectile's relative velocity
    double VpRelX = Vpx - vx;
    double VpRelY = Vpy - vy;

    // Calculate the launch angle using atan2 to get the correct quadrant
    double theta = Math.atan2(VpRelY, VpRelX);

    // Adjust the angle by 180 degrees (π radians) since the shooter is at the back
    theta += Math.PI;

    // Normalize the angle to the range [-π, π]
    theta = normalizeAngle(theta);

    // Convert angle to degrees for debugging
    double thetaDegrees = Math.toDegrees(theta);
    SmartDashboard.putNumber("Launch Angle (Radians)", theta);
    SmartDashboard.putNumber("Launch Angle (Degrees)", thetaDegrees);

    return theta;
  }

  /**
  * Normalizes an angle to the range [-pi, pi].
  *
  * @param angle Angle in radians.
  * @return Normalized angle in radians.
  */
  private double normalizeAngle(double angle) {
    while (angle > Math.PI) {
      angle -= 2 * Math.PI;
    }
    while (angle <= -Math.PI) {
      angle += 2 * Math.PI;
    }
    return angle;
  }  

  // For balance auto
  public void forwardDrive(double speed) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      new ChassisSpeeds(speed, 0, 0)
    );
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void DriverDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit, PS4Controller controller, boolean velocityControl, boolean slowAutoAim)
  { 
    if(velocityControl){
      enableVelocityControl(true);
    }
    else{
      enableVelocityControl(false);
    }
    if(controller.getSquareButton()){
      double angle = autoAim(getRobotRelativeSpeeds().vxMetersPerSecond, getRobotRelativeSpeeds().vyMetersPerSecond, getPose(), 45, target_pose.toTranslation2d());
      rot = m_botAnglePID.calculate(getPose().getRotation().getRadians(), angle);
      SmartDashboard.putNumber("Auto Aim Rotation", rot);
      if(slowAutoAim){
        xSpeed = xSpeed * 0.5;
        ySpeed = ySpeed * 0.5;
      }
    }
    drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit);
  }


  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;
    
    var swerveModuleStates =
    DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(getHeading()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
            );
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

  //   var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
  //     fieldRelative
  //         ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
  //         : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
  // SwerveDriveKinematics.desaturateWheelSpeeds(
  //     swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
  // m_frontLeft.setDesiredState(swerveModuleStates[0]);
  // m_frontRight.setDesiredState(swerveModuleStates[1]);
  // m_rearLeft.setDesiredState(swerveModuleStates[2]);
  // m_rearRight.setDesiredState(swerveModuleStates[3]);
  
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // SwerveDriveKinematics.desaturateWheelSpeeds(
    //     desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    // m_gyro.calibrate();
    m_gyro.zeroYaw();
    // m_gyro.setAngleAdjustment(180);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return -m_gyro.getYaw();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
