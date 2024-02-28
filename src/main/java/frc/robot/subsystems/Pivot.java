package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
    
    private CANSparkMax pivotMotorLeft = new CANSparkMax(PivotConstants.kPivotMotorLeftID, MotorType.kBrushless);
    private CANSparkMax pivotMotorRight = new CANSparkMax(PivotConstants.kPivotMotorRightID, MotorType.kBrushless);
    private SparkPIDController pidController;
    private AbsoluteEncoder absoluteEncoder;
    private RelativeEncoder relativeEncoder;
    ArmFeedforward feedforward = new ArmFeedforward(PivotConstants.kS, PivotConstants.kG, PivotConstants.kV, PivotConstants.kA);


    public enum PivotState {
        OFF,
        Speaker,
        AMP
    };


    public Pivot() {
        relativeEncoder = pivotMotorRight.getEncoder();
        pidController = pivotMotorRight.getPIDController();
        absoluteEncoder = pivotMotorLeft.getAbsoluteEncoder(Type.kDutyCycle);
        // pidController.setFeedbackDevice(absoluteEncoder);
        // absoluteEncoder.setPositionConversionFactor(PivotConstants.kPivotGearRatio);
        //make into constnat later need test
        // relativeEncoder.setPositionConversionFactor(1/((62/18)));

        pivotMotorLeft.setSmartCurrentLimit(40);
        pivotMotorRight.setSmartCurrentLimit(40);

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

    public void setAngleFeedFoward(double setAngle, double setAngleRate, double setAngleAccel) {
        pivotMotorLeft.setVoltage(feedforward.calculate(setAngle, setAngleRate, setAngleAccel));
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
