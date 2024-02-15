package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KitBotShooter extends SubsystemBase {
    CANSparkMax feeder;
    CANSparkMax shooter;

    public KitBotShooter() {
        feeder = new CANSparkMax(40, MotorType.kBrushless);
        feeder.setSmartCurrentLimit(40);
        shooter = new CANSparkMax(41, MotorType.kBrushless);
        shooter.setSmartCurrentLimit(40);
    }

    public void setFeeder(double speed) {
        feeder.set(speed);
    }

    public void setShooter(double speed) {
        shooter.set(speed);
    }
}
