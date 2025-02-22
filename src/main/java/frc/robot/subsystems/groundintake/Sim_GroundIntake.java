// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.groundintake;

// import org.ironmaple.simulation.SimulatedArena.Simulatable;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
// import edu.wpi.first.wpilibj.simulation.BatterySim;
// import edu.wpi.first.wpilibj.simulation.RoboRioSim;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.util.Color8Bit;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.Constants.SimulationConstants;

// public class Sim_GroundIntake extends SubsystemBase {

//   private double armKp = 1.0; // TODO: move to constants
//   private double armSetpointDegrees = 90.0; // TODO: find out if we need more of these

//   private final DCMotor armGearBox = DCMotor.getNEO(1);
//   private final PIDController controller = new PIDController(armKp, 0, 0);
//   private final Encoder encoder = new Encoder(1, 2); // TODO: find out what the heck encoder channels are for
//   private final PWMSparkMax motor = new PWMSparkMax(0); // TODO: what the heck is a motor channel

//   private final SingleJointedArmSim armSim = 
//   new SingleJointedArmSim(
//     armGearBox, // gearbox
//     12.0, //  gearing/gear ratio
//     SingleJointedArmSim.estimateMOI(
//           SimulationConstants.armLength,
//           SimulationConstants.armMass), // moment of inertia
//     0.0, // length of arm meters
//     SimulationConstants.stowedRad,
//     SimulationConstants.deployedRad,
//     true,
//     0.5 * Math.PI
//   );

//   private static final Sim_GroundIntake sim_groundIntake = new Sim_GroundIntake();

//   public static Sim_GroundIntake getInstance() {
//     return sim_groundIntake;
//   }

//   private final Mechanism2d mech2d = new Mechanism2d(40,40);
//   private final MechanismRoot2d armPivot = mech2d.getRoot("arm sim", 15, 15);
//   private final MechanismLigament2d arm =
//     armPivot.append( new MechanismLigament2d(
//         "arm",
//         20,
//         Units.radiansToDegrees(armSim.getAngleRads()),
//         5,
//         new Color8Bit(Color.kGoldenrod)
//         )
//   );

//   /** Creates a new Sim_GroundIntake. */
//   public Sim_GroundIntake() {
//     encoder.setDistancePerPulse(SimulationConstants.armEncoderDistPerPulse);

//     SmartDashboard.putData("arm sim", mech2d);

//     System.out.println(armSim.getAngleRads() + " " + Units.radiansToDegrees(armSim.getAngleRads()));
//   }

//   public Command armDeploy() {
//     var pidOutput = 
//     controller.calculate(encoder.getDistance(), Units.degreesToRadians(armSetpointDegrees));

//     return runEnd(() -> motor.setVoltage(pidOutput), () -> motor.setVoltage(0));
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     SmartDashboard.putData("arm sim", mech2d);

//     armSim.update(0.020);

//     armSim.setInput(motor.get() * RobotController.getBatteryVoltage());

//     RoboRioSim.setVInVoltage(
//       BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps())
//     );

//     arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));

//     System.out.println(armSim.hasHitLowerLimit());
//   }
// }
