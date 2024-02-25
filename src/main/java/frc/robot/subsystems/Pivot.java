package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
    
    private CANSparkMax pivotMotorLeft = new CANSparkMax(PivotConstants.kPivotMotorLeftID, MotorType.kBrushless);
    private CANSparkMax pivotMotorRight = new CANSparkMax(PivotConstants.kPivotMotorRightID, MotorType.kBrushless);
    private SparkPIDController pidController;
    private AbsoluteEncoder absoluteEncoder;
    private double position = PivotConstants.kPivotStartPosition;

    public enum PivotState {
        OFF,
        Speaker,
        AMP
    };


    public Pivot() {
        pidController = pivotMotorLeft.getPIDController();
        absoluteEncoder = pivotMotorLeft.getAbsoluteEncoder(Type.kDutyCycle);
        pidController.setFeedbackDevice(absoluteEncoder);
        absoluteEncoder.setPositionConversionFactor(PivotConstants.kPivotGearRatio);

        pivotMotorLeft.setSmartCurrentLimit(30);
        pivotMotorRight.setSmartCurrentLimit(30);

        pidController.setP(PivotConstants.kPivotP);
        pidController.setI(PivotConstants.kPivotI);
        pidController.setD(PivotConstants.kPivotD);
        pidController.setIZone(PivotConstants.kPivotIz);
        pidController.setFF(PivotConstants.kPivotFF);
        pidController.setOutputRange(PivotConstants.kPivotMinOutput, PivotConstants.kPivotMaxOutput);

        pivotMotorRight.follow(pivotMotorRight, true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Position", pivotMotorRight.getEncoder().getPosition());
    }

    public void PivotStateSetter(PivotState state){
        switch (state) {
            case OFF:
                pivotMotorLeft.set(0);
                break;
            case Speaker:
                setAngleDegree(45);
                break;
            case AMP:
                setAngleDegree(90);
                break;
        default:
            pivotMotorLeft.set(0);
            state = PivotState.OFF;
            break;
        }
    }

    //remove one of the later
    public void setPosition(double position) {
        pidController.setReference(position, ControlType.kPosition);
    }

    public void setAngleDegree(double position) {
        // pidController.setReference(position, ControlType.kPosition);
        int ur_dad = 1;

    }

}
