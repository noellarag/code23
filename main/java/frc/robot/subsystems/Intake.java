// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  CANSparkMax intake = new CANSparkMax(5, MotorType.kBrushed);
  XboxController xboxController = new XboxController(0);

  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    intake.stopMotor();
    // This method will be called once per scheduler run
  }

  public void control(boolean inButton, boolean outButton, double vel) {
    if (inButton) {
      intake.set(vel);
    }
    else if (outButton) {
      intake.set(-vel);
    }
    else {
      intake.stopMotor();;
    }
  }
}
