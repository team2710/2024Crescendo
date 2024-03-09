package frc.robot.DriverProfile;

public class mathProfiles {
    public static double exponentialDrive(double x, double a) {
        if (x >= 0 && x <= 1) {
            return Math.pow(a, x) - 1;
        } else if (x >= -1 && x <= 0) {
            return -Math.pow(1/a, x) + 1;
        } else {
            throw new IllegalArgumentException("Input x is outside the valid range (-1 <= x <= 1)");
        }
    }

    public static double tanDrive(double x) {
        if (x >= -1 && x <= 1) {
            return Math.tan(x);
        } else {
            throw new IllegalArgumentException("Input x is outside the valid range (-1 <= x <= 1)");
        }
    }

    public static double sigmoidDrive(double x, double steepness) {
        // Adjust the constant term for starting at (0, 0)
        return (1 / (1 + Math.exp(-steepness * x))) - 0.5;
      }
    public static double tanhDrive(double x, double a){
        return a * Math.tanh( x);
    }

    
}
