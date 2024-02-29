package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
    
    private CANSparkMax pivotMotorLeft = new CANSparkMax(PivotConstants.kPivotMotorLeftID, MotorType.kBrushless);
    private CANSparkMax pivotMotorRight = new CANSparkMax(PivotConstants.kPivotMotorRightID, MotorType.kBrushless);
    private SparkPIDController pidController;
    private AbsoluteEncoder absoluteEncoder;
    private RelativeEncoder relativeEncoder;
    ArmFeedforward feedforward = new ArmFeedforward(PivotConstants.kS, PivotConstants.kG, PivotConstants.kV, PivotConstants.kA);

    DriveSubsystem driveSubsystem;

    public enum PivotState {
        OFF,
        Speaker,
        AMP
    };


    public enum SpeakerColor {
        BLUE, RED
    }

    public Pivot(DriveSubsystem driveSubsystem) {
        relativeEncoder = pivotMotorRight.getEncoder();
        pidController = pivotMotorRight.getPIDController();
        absoluteEncoder = pivotMotorLeft.getAbsoluteEncoder(Type.kDutyCycle);
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

        pivotMotorLeft.setIdleMode(IdleMode.kBrake);
        pivotMotorRight.setIdleMode(IdleMode.kBrake);
        pivotMotorLeft.burnFlash();
        pivotMotorRight.burnFlash();

        // ArmAbstoRelativeSetter();
        relativeEncoder.setPosition(0);
    }

    public double distToSpeaker(Pose2d robot_pose) {
        Translation2d target = DriveConstants.blueSpeaker;
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
        Pose2d robot_pose = driveSubsystem.getPose();
        // Vector<N2> robotToTarget = VecBuilder.fill(target_pose.getX() - robot_pose.getX()  , target_pose.getY() - robot_pose.getY());
        // double dist = robotToTarget.norm() - 1;
        double dist = distToSpeaker(robot_pose)-1.2;

        // double vertDist = 2.1;
        // double angle = Math.atan(vertDist/dist) * 180 / Math.PI;
        double angle = 7.5 * dist;

        SmartDashboard.putNumber("angle", angle);

        setAngleDegree(angle);
    }

    public double angleToEncoder(double angle) {
        // y = -0.592594095866x
        return -0.592594095866 * angle;
    }

    public double distanceToAngle(double distance){
        //use desmos to find
        //x = distance
        //y = angle (the angle where note enter the speaker at that distance)
        double m = 0;
        double b = 0;
        return m * distance + b;
    }

    public void AutoAim(double distance) {
        double angle = distanceToAngle(distance);
        setAngleDegree(angle);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Position", pivotMotorRight.getEncoder().getPosition());
        SmartDashboard.putNumber("30 degrees", angleToEncoder(30));

        SmartDashboard.putNumber("Encoder", getAngle());
        SmartDashboard.putNumber("Distance to blue speaker", distToSpeaker(driveSubsystem.getPose()));
        // SmartDashboard.putNumber("Absolute as Relative", absToRelative());
    }

    public double absToRelative() {
        double offset = 0; //might need use offset depending how arm is zero in the abs encoer
        double abs_position = absoluteEncoder.getPosition();
        double position_rel = (abs_position * 1/(4 * 4 * 3));
        return position_rel  - offset; 
    }

    //will clean up later
    public void ArmAbstoRelativeSetter() {
        relativeEncoder.setPosition(absToRelative());

    }

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

    public double getAngle() {
        return relativeEncoder.getPosition();
    }

    public void setAngleFeedFowardDegree(double setAngle, double setAngleRate, double setAngleAccel) {
        pivotMotorLeft.setVoltage(feedforward.calculate(setAngle * Math.PI/180, setAngleRate * Math.PI/180, setAngleAccel * Math.PI/180));
    }

    //remove one of the later
    public void setPosition(double position) {
        SmartDashboard.putNumber("Set Position", position);
        pidController.setReference(position, ControlType.kPosition);
    }

    public void setOutput(double output) {
        pivotMotorRight.set(output);
    }

    public void setAngleDegree(double position) {
        // pidController.setReference(position, ControlType.kPosition);
        setPosition(angleToEncoder(position));

    }

}
