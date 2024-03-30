package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {

  private CANdle m_ledController = new CANdle(LEDConstants.kCANdleID);

  public enum LEDState {
    PURPLE,
    INTAKING,
    NOTE_DETECTED,
    FIRE_EFFECT,
    FLYWHEELS_ON,
    SCORE,
    ARM_IS_ZERO,
    CLIMB,
    //LOCK_IN
  };

  private LEDState m_LEDState = LEDState.PURPLE;

  public LEDSubsystem() { }

  public void setState(LEDState state) {
    m_LEDState = state;
  }

  @Override
  public void periodic() {
    switch (m_LEDState) {
      case PURPLE:
        m_ledController.clearAnimation(0);
        m_ledController.setLEDs(255, 0, 255);
        break;
      case INTAKING:
        m_ledController.clearAnimation(0);
        m_ledController.setLEDs(255, 0, 0);
        break;
      case NOTE_DETECTED:
        m_ledController.animate(new StrobeAnimation(0, 255, 0, 0, 0.6, LEDConstants.kNumLEDs));
        break;
      case FIRE_EFFECT:
        m_ledController.animate(new FireAnimation(1, 0.7, LEDConstants.kNumLEDs, 1, 1, false, 8));
        break;
      case FLYWHEELS_ON:
        m_ledController.animate(new StrobeAnimation(255, 0, 255, 0, 0.5, LEDConstants.kNumLEDs));
        break;
      case SCORE:
        //m_ledController.clearAnimation(0);
        m_ledController.animate(new StrobeAnimation(255, 255, 255, 0, 0.6, LEDConstants.kNumLEDs));
        break;
      case ARM_IS_ZERO:
        m_ledController.animate(new RainbowAnimation(1, 0.35, LEDConstants.kNumLEDs));
        break;
      case CLIMB:
        m_ledController.animate(new RainbowAnimation(1, 0.95, LEDConstants.kNumLEDs));
      // case LOCK_IN:
      //   m_ledController.animate(new StrobeAnimation(0, 191, 255, 0, 0.8, LEDConstants.kNumLEDs))
    }

    Logger.recordOutput("LEDState", m_LEDState);

    
  }

}
