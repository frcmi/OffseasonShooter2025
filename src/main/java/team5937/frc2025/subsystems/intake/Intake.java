// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5937.frc2025.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import team5937.lib.subsystem.VirtualSubsystem;
import team5937.lib.subsystem.angular.AngularIO;
import team5937.lib.subsystem.angular.AngularSubsystem;
import team5937.lib.subsystem.linear.LinearIO;
import team5937.lib.subsystem.linear.LinearSubsystem;
import team5937.frc2025.constants.Intake.RollerConstants;
import team5937.frc2025.constants.Intake.PivotConstants;
import lombok.Getter;

public class Intake extends VirtualSubsystem {
  private final AngularSubsystem rollers;
  private final AngularSubsystem pivot;

  @Getter private IntakeState targetState = new IntakeState(Degrees.of(0), Volts.of(0));
  @Getter private IntakeState measuredState = targetState;
  /** Creates a new Intake. */
  public Intake() {
        this(
                new AngularSubsystem(
                        new AngularIO() {}, RollerConstants.kSubsystemConfigReal),
                new AngularSubsystem(
                        new AngularIO() {}, PivotConstants.kSubsystemConfigReal));
    }
  public Intake(AngularSubsystem rollers, AngularSubsystem pivot) {
    this.rollers = rollers;
    this.pivot = pivot;

    pivot.setDefaultCommand(pivot.holdAtGoal(() -> getTargetState().getPivot()));
    rollers.setDefaultCommand(rollers.openLoop(Volts.of(0)));

    measuredState = targetState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    measuredState.setPivot(pivot.getAngle());
    measuredState.setRollers(targetState.getRollers());

    Logger.recordOutput("Intake/TargetState", targetState);
    Logger.recordOutput("Intake/MeasuredState", measuredState);
  }
}
