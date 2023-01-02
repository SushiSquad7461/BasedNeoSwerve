package frc.robot.utils;

public class AngleUtils {
  // Stolen nearly verbatim from a CD thread: https://www.chiefdelphi.com/t/wrap-around-with-rev-spark-maxes/403608/13
  /**
   * Converts a value ranged from [0, 2PI] to an unbound value based off the 
   * anchor.
   * 
   * Prevents Spark from rotating from 2PI to 0.25PI when it can rotate to 
   * 2.25PI.
   */
  public static double reboundValue(double value, double anchor) {
    double lowerBound = anchor - Math.PI;
    double upperBound = anchor + Math.PI;
        
    if (value < lowerBound) {
      value = upperBound + ((value - lowerBound) % (upperBound - lowerBound));
    } else if (value > upperBound) {
      value = lowerBound + ((value - upperBound) % (upperBound - lowerBound));
    }
    
    return value;
  }
}