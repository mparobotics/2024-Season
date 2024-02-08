package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class OnboardModuleState {
  //modified modulo function that actually works properly with negative numbers.  e.g.  -8 % 6 == -2  but fixedMod(-8,6) = 4
  public static double fixedMod(double a, double b){
    double bad = a % b;
    return bad + (bad < 0? b: 0);
  }
  public static double smolOptimize180(double currentValue, double targetValue){
    return currentValue + fixedMod(targetValue - currentValue -180, 360) - 180;
  }
  //for a given target angle, find the closest equivalent angle to the module's current direction
  //custom optimize function because built-in doesn't work for some reason
  //see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html for more info
  public static SwerveModuleState smolOptimize(SwerveModuleState desiredState, Rotation2d currentAngle){
    //the module's current direction
    double current = currentAngle.getDegrees();
    //the direction you want to drive in
    double target = desiredState.angle.getDegrees();

    //find the direction that points you in the target direction with the least angle change
    double error = smolOptimize180(current, target);
    //sometimes we can reverse the drive motor to avoid turning 180 degress.
    //for example, if you were pointing 0 degrees straight ahead, and you suddenly wanted to go 175 degrees counterclockwise(almost backwards)
    //you could just turn 5 degrees clockwise and drive the motor backwards -- and reach your target angle much faster

    int reverseSpeed = 1;
    if(error > 90){
      error -= 180;
      reverseSpeed = -1;
    }
    if(error < -90){
      error = 180;
      reverseSpeed = -1;
    }

    

    double speed = desiredState.speedMetersPerSecond * reverseSpeed;
    return new SwerveModuleState(speed,Rotation2d.fromDegrees(current + error));
  }
  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to include placing in
   * appropriate scope for CTRE and REV onboard control as both controllers as of writing don't have
   * support for continuous input.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) 
  {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees(); //how far module needs to travel
    
    //if needs to travel more than +/- 90 degrees, we can optimize
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed; //reverse wheel direction
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
      //set target angle if the delta is +90 by subracting 180, if the delta is -90 add 180 to target angle
      //reversing the wheel meens the turn modules must travel less
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound; 
    double upperBound;
    double lowerOffset = scopeReference % 360; //what is the degrees in 0 to 360
    //modulo (%) finds the remainder. for example, 450%360 is 90, 450˚ is the same as 90˚

    //modulos are funky and output differently between coding languages. 
    //the tldr of this https://en.wikipedia.org/wiki/Modulo wikipedia page is that i think loweroffset
    //will be negative when scopeReference is negative. 
    //this varies by coding languages though and is dumb and annoying
    if (lowerOffset >= 0) {
      //lowerbound is now where the module is minus the offset. This creates the nearest "bound" of 360 degree ranges
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
      //upperbound is 360 higher than that
    } else {
      //if the angle is currently negative, we have to reverse the signs! yay!
      //this is very painful to wrap my head around
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }

    //makes sure new angle calculated is between -360 and 360
    while (newAngle < lowerBound) {
      //keep addin 360 if not in bounds
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      //keep subtractin it if its too high
      newAngle -= 360;
    }

    //but wait, there's more! remember, we can always just reverse a modules direction
    //thus, if its greater than 180, we can subtract or add 180 to make it a lower angle to turn to
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }
}