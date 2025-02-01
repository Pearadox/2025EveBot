// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundintake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.ironmaple.simulation.SimulatedArena.Simulatable;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

import frc.robot.Constants.SimulationConstants;
import frc.lib.drivers.PearadoxSparkMax;

public class GroundIntakeSim extends SubsystemBase {

  public DCMotor groundIntakeGearBox = DCMotor.getNEO(1);

  private PIDController controller;

  private final SingleJointedArmSim GroundIntakeSim = new SingleJointedArmSim(
          groundIntakeGearBox,
          12,
          SingleJointedArmSim.estimateMOI(SimulationConstants.armLength, SimulationConstants.armMass),
          SimulationConstants.armLength,
          Units.degreesToRadians(-180), //90
          Units.degreesToRadians(180), //210
          false,
          Units.degreesToRadians(-90));

  private final Mechanism2d mech2d = new Mechanism2d(5, 5); // TODO: find width and height
  private final MechanismRoot2d intakePivot = mech2d.getRoot("intakePivot", 1.5, 2.5);
  private final MechanismLigament2d intake = intakePivot.append(
    new MechanismLigament2d("intake", SimulationConstants.armLength, SimulationConstants.stowedDeg, 3, new Color8Bit(Color.kGold)));

  public static final GroundIntakeSim simIntake = new GroundIntakeSim();

  public static GroundIntakeSim getInstance(){
    return simIntake;
  }

  public enum SimPivotMode {
    stowed, shooting, deployed
  }

  public SimPivotMode simPivotPos = SimPivotMode.stowed;

  /** Creates a new GroundIntakeSim. */
  public GroundIntakeSim() {
    SmartDashboard.putData("Intake Sim", mech2d);
    GroundIntakeSim.setState(SimulationConstants.stowedDeg, 0.0);
    controller = new PIDController(0.25, 0.0, 0.0);
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData("Intake Sim", mech2d);

  }

  public void setAngle(double angle) {
    double intendedAngle = angle;

    double volts = MathUtil.clamp(
      controller.calculate(GroundIntakeSim.getAngleRads(), intendedAngle),
      -12, 12);

    GroundIntakeSim.setInputVoltage(volts);
  }

  public void setDeployed() {
    simPivotPos = SimPivotMode.deployed;
  }
  
  public void setShooting() {
    simPivotPos = SimPivotMode.shooting;
  }
  public void setStowed() {
    simPivotPos = SimPivotMode.stowed;
  }

  public void simIntakeHold() {
    if (simPivotPos == SimPivotMode.stowed) {
      setAngle(SimulationConstants.stowedDeg);
    } else if (simPivotPos == SimPivotMode.deployed) {
      setAngle(SimulationConstants.deployedDeg);
    } else if (simPivotPos == SimPivotMode.shooting) {
      setAngle(SimulationConstants.shootingDeg);
    }
  }

  public SimPivotMode getSimPivotPos() {
    return simPivotPos;
  }

}
