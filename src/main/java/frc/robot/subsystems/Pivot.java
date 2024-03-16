package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.RobotContainer;

public class Pivot extends SubsystemBase {
    
    private CANSparkMax pivotMotorLeft = new CANSparkMax(PivotConstants.kPivotMotorLeftID, MotorType.kBrushless);
    private CANSparkMax pivotMotorRight = new CANSparkMax(PivotConstants.kPivotMotorRightID, MotorType.kBrushless);
    private SparkPIDController pidController;
    // private AbsoluteEncoder absoluteEncoder;
    private RelativeEncoder relativeEncoder;
    ArmFeedforward feedforward = new ArmFeedforward(PivotConstants.kS, PivotConstants.kG, PivotConstants.kV, PivotConstants.kA);
    Translation3d target = DriveConstants.blueSpeaker;
    DriveSubsystem driveSubsystem;
    Rotation3d armRotation3d = new Rotation3d(0,0,0);

    Pose3d pivotPose = new Pose3d(-0.234442,0,0.1905, armRotation3d);

    private double setAngle = 0;


    double[] armAngle3DArray = new double[]{0,0,0,0,0,0,0};



    public enum PivotState {
        OFF,
        Speaker,
        AMP
    };


    public enum SpeakerColor {
        BLUE, RED
    }

    private LEDSubsystem m_ledSubsystem;

    public Pivot(DriveSubsystem driveSubsystem, LEDSubsystem ledSubsystem) {
        m_ledSubsystem = ledSubsystem;
        relativeEncoder = pivotMotorRight.getEncoder();
        pidController = pivotMotorRight.getPIDController();
        // absoluteEncoder = pivotMotorLeft.getAbsoluteEncoder(Type.kDutyCycle);
        this.driveSubsystem = driveSubsystem;
        // pidController.setFeedbackDevice(absoluteEncoder);
        // absoluteEncoder.setPositionConversionFactor(PivotConstants.kPivotGearRatio);
        //make into constnat later need test
        // relativeEncoder.setPositionConversionFactor(1/((62/18)));

        pivotMotorLeft.setSmartCurrentLimit(20);
        pivotMotorRight.setSmartCurrentLimit(20);

        pidController.setP(PivotConstants.kPivotP);
        pidController.setI(PivotConstants.kPivotI);
        pidController.setD(PivotConstants.kPivotD);
        pidController.setIZone(PivotConstants.kPivotIz);
        pidController.setFF(PivotConstants.kPivotFF);
        pidController.setOutputRange(PivotConstants.kPivotMinOutput, PivotConstants.kPivotMaxOutput);
       
        pivotMotorLeft.follow(pivotMotorRight, true);

        SmartDashboard.putBoolean("Brake Mode", false);

        pivotMotorLeft.setIdleMode(IdleMode.kCoast);
        pivotMotorRight.setIdleMode(IdleMode.kCoast);
        pivotMotorLeft.burnFlash();
        pivotMotorRight.burnFlash();

        // ArmAbstoRelativeSetter();
        relativeEncoder.setPosition(0);
    }

    public double distToSpeaker(Pose2d robot_pose) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return 0;
        switch (alliance.get()) {
            case Red:
                target = DriveConstants.redSpeaker;
                break;
            case Blue:
                target = DriveConstants.blueSpeaker;
                break;
            default:
                break;
        }
        Vector<N2> robotToTarget = VecBuilder.fill(target.getX() - robot_pose.getX(), target.getY() - robot_pose.getY());
        double dist = robotToTarget.norm();
        return dist;
    }

    public void autoAimPivot() {
        double angle = Math.atan(2.1/distToSpeaker(RobotContainer.m_robotDrive.getPose())) + PivotConstants.shooterAngle - Math.asin((Math.sin(PivotConstants.shooterAngle) * PivotConstants.armLength))/(RobotToTarget3D());
        // SmartDashboard.putNumber("Angle to Target", angle * 180 / Math.PI);
        double clampAngle= MathUtil.clamp(180 - (angle * (180 / Math.PI)), 0, 70);
        // SmartDashboard.putNumber("auto aim angle", clampAngle);
        setAngleDegree(clampAngle);
    }

    public double autoAimPivotDeg() {
        double angle = Math.atan(2.1/distToSpeaker(RobotContainer.m_robotDrive.getPose())) + PivotConstants.shooterAngle - Math.asin((Math.sin(PivotConstants.shooterAngle) * PivotConstants.armLength))/(RobotToTarget3D());
        // SmartDashboard.putNumber("Angle to Target", angle * 180 / Math.PI);
        double clampAngle= MathUtil.clamp(180 - (angle * (180 / Math.PI)), 0, 70);
        // SmartDashboard.putNumber("auto aim angle", clampAngle);
        return clampAngle;
    }


    public double RobotToTarget3D(){
        Translation3d robot_pose = new Translation3d(RobotContainer.m_robotDrive.getPose().getX(), RobotContainer.m_robotDrive.getPose().getY(), 0);
        return robot_pose.getDistance(target);

    }

    public Command autoAimPivotCommand() {
        return new InstantCommand(() -> {
            autoAimPivot();
        }, this);
    }

    public double angleToEncoder(double angle) {
        // y = -0.592594095866x
        return -0.592594095866 * angle;
    }

    public double encoderToAngle(double encoder) {
        return -1.6874957192 * encoder;
    }


    public boolean reachedSetpoint(){
        return MathUtil.isNear(autoAimPivotDeg(), getAngle(), 3);
    }

    @Override
    public void periodic() {
        armRotation3d = new Rotation3d(0, -getAngle() * Math.PI/180 ,0);

        SmartDashboard.putNumberArray("arm 3d", armAngle3DArray);
        SmartDashboard.putNumber("Pivot Angle", getAngle());
        SmartDashboard.putNumber("Distance to speaker", distToSpeaker(driveSubsystem.getPose()));

        Logger.recordOutput("MyPose3d", pivotPose);
        Logger.recordOutput("Arm/Angle", getAngle());

        boolean brakeMode = SmartDashboard.getBoolean("Brake Mode", true);
        if (MathUtil.isNear(PivotConstants.kPivotStow, getAngle(), 5) && !brakeMode) {
            brakeMode = true;
            SmartDashboard.putBoolean("Brake Mode", brakeMode);
        }
        if (brakeMode && pivotMotorLeft.getIdleMode() == IdleMode.kCoast) {
            pivotMotorLeft.setIdleMode(IdleMode.kBrake);
            pivotMotorRight.setIdleMode(IdleMode.kBrake);
        } else if (!brakeMode && pivotMotorLeft.getIdleMode() == IdleMode.kBrake) {
            pivotMotorLeft.setIdleMode(IdleMode.kCoast);
            pivotMotorRight.setIdleMode(IdleMode.kCoast);
        }

        if (MathUtil.isNear(0, getAngle(), 5) && DriverStation.isDisabled()) {
            m_ledSubsystem.setState(LEDState.ARM_IS_ZERO);
        }
    }

    // public double absToRelative() {
    //     double offset = 0; //might need use offset depending how arm is zero in the abs encoer
    //     double abs_position = absoluteEncoder.getPosition();
    //     double position_rel = (abs_position * 1/(4 * 4 * 3));
    //     return position_rel  - offset; 
    // }

    //will clean up later
    // public void ArmAbstoRelativeSetter() {
    //     relativeEncoder.setPosition(absToRelative());

    // }

    public void PivotStateSetter(PivotState state){
        switch (state) {
            case OFF:
                pivotMotorRight.set(0);
                break;
            case Speaker:
                setAngleDegree(45);
                break;
            case AMP:
                setAngleDegree(90);
                break;
        default:
            pivotMotorRight.set(0);
            state = PivotState.OFF;
            break;
        }
    }

    public Command pivotMoveCommand(double position) {
        return new InstantCommand(() -> {
            setAngleDegree(position);
        }, this);
    }

    public double getAngle() {
        return encoderToAngle(relativeEncoder.getPosition());
    }

    public void setAngleFeedFowardDegree(double setAngle, double setAngleRate, double setAngleAccel) {
        pivotMotorLeft.setVoltage(feedforward.calculate(setAngle * Math.PI/180, setAngleRate * Math.PI/180, setAngleAccel * Math.PI/180));
    }

    //remove one of the later
    public void setPosition(double position) {
        pidController.setReference(position, ControlType.kPosition);
    }

    public void setOutput(double output) {
        pivotMotorRight.set(output);
    }


    // kill the pivot rip
    public void disable() {
        pivotMotorRight.set(0);
        pivotMotorLeft.setIdleMode(IdleMode.kCoast);
        pivotMotorRight.setIdleMode(IdleMode.kCoast);
        
    }

    public void enable() {
        pivotMotorLeft.setIdleMode(IdleMode.kBrake);
        pivotMotorRight.setIdleMode(IdleMode.kBrake);
        zeroPivot();
    }

    public void zeroPivot(){
        relativeEncoder.setPosition(0);
    }



    public void setAngleDegree(double position) {
        if (position > 80) position = 80;
        if (position < 0) position = 0;
        setAngle = position;
        // pidController.setReference(position, ControlType.kPosition);
        setPosition(angleToEncoder(position));

    }

}
