// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.lightstripConstants;
import frc.robot.lightstrip.LedState;
import frc.robot.lightstrip.TempLedState;

public class Lightstrip extends SubsystemBase {
  CANdle candle = new CANdle(lightstripConstants.candlePort);
  RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, lightstripConstants.ledCount + 8);

  private LedState defaultColor = lightstripConstants.defaultState;
  private Timer defaultTimer = new Timer();
  private LedState currentColor = null;
  private Timer currentTimer = new Timer();
  private TempLedState tempColor = null;
  private Timer tempTimer = new Timer();

  private boolean isDefault = false;

  /** Creates a new Lightstrip. */
  public Lightstrip() {
    defaultTimer.reset();
    defaultTimer.start();

    isDefault = false;

    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.75;
    candle.configAllSettings(config); 
  }

  @Override
  public void periodic() {
    if(isDefault) {
      return;
    }

    if(tempColor != null) {
      if(tempColor.getSeconds() < tempTimer.get()) {
        tempColor = null;
        tempTimer.stop();
      } else {
        update(toLedState(tempColor), tempTimer);
      }
    } else if(currentColor != null) {
      update(currentColor, currentTimer);
    } else {
      update(defaultColor, defaultTimer);
    }

    if(currentColor != null) {
      SmartDashboard.putNumberArray("Current", currentColor.getState());
    }

    if(tempColor != null) {
      SmartDashboard.putNumberArray("Temp", tempColor.getState());
    }
  }

  public void setDefault(boolean defaultState) {
    isDefault = defaultState;

    if(isDefault) {
      candle.animate(rainbowAnim);
    }
  }

  private LedState toLedState(TempLedState state) {
    return new LedState(state.getRed(), state.getGreen(), state.getBlue(), state.getEffect());
  }

  private void update(LedState state, Timer timer) {
    SmartDashboard.putNumber("Timer", timer.get() % 1 - 0.50 * state.getRed());
    if(state.getEffect() == "Solid") {
      setColor(state.getRed(), state.getGreen(), state.getBlue());
    } else if(state.getEffect() == "Blink") {
      if((timer.get() % 2) < 1) {
        setColor(state.getRed(), state.getGreen(), state.getBlue());
      } else if((timer.get() % 2) >= 1) {
        setColor(0, 0, 0);
      }
    } else if(state.getEffect() == "Fast Blink") {
      if((timer.get() % 0.5) < 0.25) {
        setColor(state.getRed(), state.getGreen(), state.getBlue());
      } else if((timer.get() % 0.5) >= 0.25) {
        setColor(0, 0, 0);
      }
    }
  }

  public void toggleOnColor(LedState state) {
    setColor(state);
  }

  public void toggleOffColor(LedState state) {
    if(currentColor != null && state.compare(currentColor)) {
      currentColor = null;
      currentTimer.stop();
    }
  }

  private void resetCurrent() {
    currentTimer.reset();
    currentTimer.start();
  }

  public void setColor(LedState state) {
    if(currentColor == null) {
      currentColor = state;
      resetCurrent();
    } else if(!state.compare(currentColor)) {
      currentColor = state;
      resetCurrent();
    }
  }

  public void tempColor(TempLedState state) {
    tempColor = state;
    resetTemp();
  }

  private void resetTemp() {
    tempTimer.reset();
    tempTimer.start();
  }

  private void setColor(int red, int green, int blue) {
    candle.setLEDs(red, green, blue, 0, 0, lightstripConstants.ledCount + 8);
  }
}
