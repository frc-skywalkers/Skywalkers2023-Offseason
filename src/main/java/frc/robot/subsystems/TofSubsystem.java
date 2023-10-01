// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TofConstants;

import com.playingwithfusion.TimeOfFlight;

public class TofSubsystem extends SubsystemBase {
  private final TimeOfFlight primaryTof = new TimeOfFlight(TofConstants.primaryPort);
  private final TimeOfFlight secondaryTof = new TimeOfFlight(TofConstants.secondaryPort);

  /** Creates a new TofSubsystem. */
  public TofSubsystem() {
    primaryTof.setRangingMode(TimeOfFlight.RangingMode.Short, 24); //short is default, sampling rate could be between 24 and 1000 ms
    primaryTof.setRangeOfInterest(0, 0, 16, 16); //16 by 16 should alr be default

    secondaryTof.setRangingMode(TimeOfFlight.RangingMode.Short, 24); //short is default, sampling rate could be between 24 and 1000 ms
    secondaryTof.setRangeOfInterest(0, 0, 16, 16); //16 by 16 should alr be default
  }

  public double getRawPrimary() {
    return primaryTof.getRange();
  }

  public double getRawSecondary() {
    return secondaryTof.getRange();
  }

  public double getConeOffset() {
    return ((getConeWidth() / 2) + getRawPrimary()) - TofConstants.halfway.cone;
  }

  public double getCubeOffset() {
    if(TofConstants.secondaryMode) {
      return 0; //no cube tof, just return 0 for no offset from middle
    } else {
      return ((getCubeWidth() / 2) + getRawSecondary()) - TofConstants.halfway.cone;
    }
  }

  public double getConeWidth() {
    if(TofConstants.secondaryMode) {
      return (TofConstants.halfway.cone * 2) - (getRawPrimary() + getRawSecondary());
    } else {
      return TofConstants.width.cone;
    }
  }

  public double getCubeWidth() {
    return TofConstants.width.cube;
  }

  public boolean isCubeTofOn() {
    return !TofConstants.secondaryMode;
  }

  public boolean isCubeHeld() {
    return getRawSecondary() < TofConstants.full.cube;
  }

  public boolean isConeHeld() {
    return getRawPrimary() < TofConstants.full.cone;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Primary Time of Flight", primaryTof.getRange());
    SmartDashboard.putNumber("Secondary Time of Flight", secondaryTof.getRange());
    SmartDashboard.putBoolean("Tof Cube Held", isCubeHeld());
    SmartDashboard.putBoolean("Tof Cube Held", isConeHeld());
    SmartDashboard.putNumber("Tof Cone Width", getConeWidth());
    SmartDashboard.putNumber("Tof Cube Width", getCubeWidth());
    SmartDashboard.putNumber("Tof Cone Offset", getConeOffset());
    SmartDashboard.putNumber("Tof Cube Offset", getCubeOffset());
  }
}