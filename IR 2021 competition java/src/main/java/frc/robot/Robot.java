// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
   
import com.ctre.phoenix.motorcontrol.ControlMode;   //include vendor libraries
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;  //include WPILib libraries
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.POVButton;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.cameraserver.CameraServer;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  Joystick driveStick = new Joystick(0);  //joystick setup
  Joystick buttonPad = new Joystick(1); 

  POVButton povButtonUp = new POVButton(driveStick, 0);
  POVButton povButtonDown = new POVButton(driveStick, 180);
  POVButton povButtonLeft = new POVButton(driveStick, 270);
  POVButton povButtonRight = new POVButton(driveStick, 90);
  
  TalonSRX flDrive = new TalonSRX(0);   //setup drive motors; talons via CAN
  TalonSRX blDrive = new TalonSRX(1);
  TalonSRX frDrive = new TalonSRX(2);
  TalonSRX brDrive = new TalonSRX(3);

  Spark cpSpinner = new Spark(1);             //setup spark/sparkMax controllers 
  PWMSparkMax winch = new PWMSparkMax(2);
  PWMSparkMax flyWheel = new PWMSparkMax(3);
  Spark intakeRollers = new Spark(4);
  Spark intakeArm = new Spark(5);
  Spark telescope = new Spark(6);
  Spark carousel = new Spark(7);
  Spark shooterIntake = new Spark(8);
  Spark blinkin = new Spark(9);   //setup Blinkin LED Strip controller. Yes it uses the spark class.
                                  // see https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf for documentation

  Compressor compressor;   //setup pnuematics
  DoubleSolenoid shooterIntakeActuator = new DoubleSolenoid(0,1);
  DoubleSolenoid cpSpinnerActuator = new DoubleSolenoid(2,3);

  Timer shootDelayTimer = new Timer(); //setup various timers

  boolean ballsHaveBeenShot = false;
  
  @Override
  public void robotInit() {

    CameraServer.getInstance().startAutomaticCapture();

    flDrive.setInverted(true);
    blDrive.setInverted(true);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
      shootDelayTimer.reset();
  }
  @Override
  public void autonomousPeriodic() {
    
    // The angle offset to the target on the horizontal axis.
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    // The angle offset to the target on the vertical axis.
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    if (-1 >= tx || tx >= 1 || -1 >=ty || ty >= 1 ){
      if (ballsHaveBeenShot == false) {
    //variable setup
    double xKP = -0.1;
    double xKD = 0.05;
    double xSetpoint = 0;
    double xLastError = 0;

    double yKP = -0.1;
    double yKD = 0.05;
    double ySetpoint = 0;
    double yLastError = 0;

    double lastTimestamp = 0;

    //calculations
    double timeInterval = Timer.getFPGATimestamp() - lastTimestamp;

    double xError = xSetpoint - tx;
    double xErrorRate = (xError - xLastError) / timeInterval;
    double xOutputSpeed = xKP * xError + xKD * xErrorRate;
    xLastError = xError;

    double yError = ySetpoint - ty;
    double yErrorRate = (yError - yLastError) / timeInterval;
    double yOutputSpeed = yKP * yError + yKD * yErrorRate;
    yLastError = yError;

    lastTimestamp = Timer.getFPGATimestamp();

    //combine x and y outputs
    double left = yOutputSpeed - xOutputSpeed;
    double right = yOutputSpeed + xOutputSpeed;

    //motor output
    flDrive.set(ControlMode.PercentOutput, left);
    blDrive.set(ControlMode.PercentOutput, left);
    frDrive.set(ControlMode.PercentOutput, right);
    brDrive.set(ControlMode.PercentOutput, right);
      }
       else if (ballsHaveBeenShot == true) {

      }
    }

    double time = shootDelayTimer.get();

    if (-1 <= tx && tx <= 1 && -1 <=ty && ty <= 1) {
      shootDelayTimer.start();
      flyWheel.set(0.62);
      if (time >= 1 && time <=5) {
        flDrive.set(ControlMode.PercentOutput, 0);
        blDrive.set(ControlMode.PercentOutput, 0);
        frDrive.set(ControlMode.PercentOutput, 0);
        brDrive.set(ControlMode.PercentOutput, 0);
        shooterIntakeActuator.set(kReverse); //extend actuator; down pos
        shooterIntake.set(-1);
        carousel.set(.32);
        ballsHaveBeenShot = true;
      }
      else if (time >= 5 && time < 6) {
        
        shooterIntakeActuator.set(kForward);
        shooterIntake.set(0);
        carousel.set(0);
        flyWheel.set(0);
        frDrive.set(ControlMode.PercentOutput, -.2);
        brDrive.set(ControlMode.PercentOutput, -.2);
      }
      else if (time >=6 && time <8) {
        flDrive.set(ControlMode.PercentOutput, 0);
        blDrive.set(ControlMode.PercentOutput, 0);
        frDrive.set(ControlMode.PercentOutput, 0);
        brDrive.set(ControlMode.PercentOutput, 0);}
        /*flDrive.set(ControlMode.PercentOutput, -.4);
        blDrive.set(ControlMode.PercentOutput, -.4);
        frDrive.set(ControlMode.PercentOutput, -.4);
        brDrive.set(ControlMode.PercentOutput, -.4);
      }
      else if (time >= 9) {
        flDrive.set(ControlMode.PercentOutput, 0);
        blDrive.set(ControlMode.PercentOutput, 0);
        frDrive.set(ControlMode.PercentOutput, 0);
        brDrive.set(ControlMode.PercentOutput, 0);
      }*/
    }
    
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

// flywheel
    if (driveStick.getRawButton(1)) {flyWheel.set(0.62);} //shooting mode
    else if (driveStick.getRawButton(2)) {flyWheel.set(-.3);} //reversde mode: to clear jams
    else {flyWheel.set(0);}
// intake rollers
    if (buttonPad.getRawButton(12)) {intakeRollers.set(-1);}  //inverted, intaking
    else if (buttonPad.getRawButton(9)) {intakeRollers.set(1);} //"reversing", outputting
    else {intakeRollers.set(0);}
// intake arm
    if (buttonPad.getRawButton(10)) {intakeArm.set(-.5);} //inverted, deploying; doesnt need full power (especially since gravity helps)
    else if (buttonPad.getRawButton(7)) {intakeArm.set(.6);}  //retracting; little more power to get back up
    else {intakeArm.set(0);}
// carousel (indexer, wheel of fortune)
    if (buttonPad.getRawButton(11)) {carousel.set(.35);}  //CCW; slow down... will lose control, might break
    else if (buttonPad.getRawButton(8)) {carousel.set(-.35);} //CW
    else {carousel.set(0);}
// shooter intake, transitionary stage
    if (driveStick.getRawButton(3)) {
      shooterIntakeActuator.set(kReverse); //extend actuator; down pos
      shooterIntake.set(-1);}  //rveresed; intaking to shooter
    else if (driveStick.getRawButton(4))  {
      shooterIntakeActuator.set(kForward); //retract actuator; up pos
      shooterIntake.set(.7);}
    else {
      shooterIntakeActuator.set(kForward); //retract actuator; up pos
      shooterIntake.set(0);}
// climber winch
    if (buttonPad.getRawButton(2))  {winch.set(-1);}  //WINCH MUST ONLY SPIN BACKWARDS to save the motor and built in ratchet
    else {winch.set(0);}
//Control Panel (CP) Spinner
    if (povButtonUp.get()) {cpSpinnerActuator.set(kReverse);} //raises motor. Is held ther until the "down" button is pressed
    if (povButtonDown.get()) {cpSpinnerActuator.set(kForward);} //this is the down config and starting. only returns to this pos when button is pressed
    if (povButtonLeft.get()) {cpSpinner.set(-.2);}  //spin left
    else if (povButtonRight.get()) {cpSpinner.set(.2);} //spin right
    else {cpSpinner.set(0);}  //default state

//Drive control
    if (buttonPad.getRawButton(1)) {
      // The angle offset to the target on the horizontal axis.
      double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      // The angle offset to the target on the vertical axis.
      double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
      // limelight x & y PD loop
      //variable setup
      double xKP = -0.1;
      double xKD = 0.1;
      double xSetpoint = 0;
      double xLastError = 0;

      double yKP = -0.1;
      double yKD = 0.1;
      double ySetpoint = 0;
      double yLastError = 0;

      double lastTimestamp = 0;

      //calculations
      double timeInterval = Timer.getFPGATimestamp() - lastTimestamp;

      double xError = xSetpoint - tx;
      double xErrorRate = (xError - xLastError) / timeInterval;
      double xOutputSpeed = xKP * xError + xKD * xErrorRate;
      xLastError = xError;

      double yError = ySetpoint - ty;
      double yErrorRate = (yError - yLastError) / timeInterval;
      double yOutputSpeed = yKP * yError + yKD * yErrorRate;
      yLastError = yError;

      lastTimestamp = Timer.getFPGATimestamp();

      //combine x and y outputs
      double left = yOutputSpeed - xOutputSpeed;
      double right = yOutputSpeed + xOutputSpeed;

      //motor output
      flDrive.set(ControlMode.PercentOutput, left);
      blDrive.set(ControlMode.PercentOutput, left);
      frDrive.set(ControlMode.PercentOutput, right);
      brDrive.set(ControlMode.PercentOutput, right);
    }
    else {
      double speed = driveStick.getY();
      double turn = driveStick.getZ() * -.4;

      double left = speed + turn;
      double right = speed - turn;

      flDrive.set(ControlMode.PercentOutput, left);
      blDrive.set(ControlMode.PercentOutput, left);
      frDrive.set(ControlMode.PercentOutput, right);
      brDrive.set(ControlMode.PercentOutput, right);
    }
  
  
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
