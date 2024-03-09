// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class VisionConstants {
    public static final double kLimelightLatency = 0;
  }

  public static final class EndEffectorConstants {
    public static final int kIntakeMotorID = 44;
    public static final int kFlywheelMotorTopID = 42;
    public static final int kFlywheelMotorBottomID = 43;

    public static final double kIntakeSpeed = 1;
    public static final double kFlywheelMotorSpeed = 6300;
    
    public static final double kFlywheelP = 0.002;
    public static final double kFlywheelI = 0;
    public static final double kFlywheelD = 0.0001;
    public static final double kFlywheelIz = 0;
    public static final double kFlywheelFF = 1f/6784f;
    public static final double kFlywheelMinOutput = -1;
    public static final double kFlywheelMaxOutput = 1;
  }

  public static final class PivotConstants {

    public static final int kPivotMotorLeftID = 21;
    public static final int kPivotMotorRightID = 20;

    public static final double kPivotPodiumAngle = 20;
    public static final double kPivotWingAngle = 25;
    public static final double kPivotAmpAngle = 80;
    public static final double kPivotZero = 0;
    public static final double kPivotStow = 72;
    
    
    public static final double kS = 0.01;
    public static final double kG = 0.39;
    public static final double kV = 0.06;
    public static final double kA = 0;
    //calulcations
    //https://www.reca.lc/arm?armMass=%7B%22s%22%3A14%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A27.784294%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=80&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A165.333333%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D

    public static final double kPivotGearRatio = 18.0/62.0;
    public static final double kPivotP = 0.04;
    public static final double kPivotI = 0;
    public static final double kPivotD = 0.011;
    public static final double kPivotIz = 0;
    public static final double kPivotFF = 0;
    public static final double kPivotMinOutput = -0.825;
    public static final double kPivotMaxOutput = 0.3;

    public static final double kPivotStartPosition = 0;
    public static double shooterAngle  = (135) * Math.PI/180;
    public static final double armLength = 0.920; // Meters


  }

  public static final class ClimberConstants {
    public static final int kClimberMotorID = 30;
    public static final double kMaxPosition = 1;
    public static final double kMinPosition = 0;
    public static final double kUpSpeed = 1;
    public static final double kDownSpeed = -1;
  }

  public static final class DriveConstants {
    public static final Translation3d blueSpeaker = new Translation3d(0.225, 5.55,2.1);
    public static final Translation3d redSpeaker = new Translation3d(16.317, 5.55,2.1);
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 6.8
    ;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    public static final double kDriveWidth = 26.5;

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(kDriveWidth);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(kDriveWidth);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 32;
    public static final int kRearLeftTurningCanId = 16;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 12;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = 6000 / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = 3.56;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelCircumferenceMeters)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelCircumferenceMeters)
        / kDrivingMotorReduction); // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.007;
    public static final double kDrivingI = 0.341146341;
    public static final double kDrivingD = 0.0000095;
    public static final double kDrivingFF = 0;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 0.8;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 80; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kAuxControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kSwerveDiscreteTimestep = 0.02;
    public static final double kSwerveDriveRadiusMeters = Units.inchesToMeters(DriveConstants.kDriveWidth) / 2;
    public static final PIDConstants kTranslationPIDConstants = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants kRotationPIDConstants = new PIDConstants(5.0, 0.0, 0.0);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
