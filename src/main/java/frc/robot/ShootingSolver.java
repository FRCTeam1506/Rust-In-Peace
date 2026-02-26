package frc.robot;

//Just the math
public class ShootingSolver {
    public static class ShooterState {
        public final double angleRad;
        public final double bottomRPM;
        public final double topRPM;
        public final double tof;

        public ShooterState(double a, double br, double tr, double t) {
            angleRad = a;
            bottomRPM = br;
            topRPM = tr;
            tof = t;
        }
    }

    public static ShooterState solve(double distMeters) {
        final double g = 9.806;
        final double h = Constants.ShotConstants.TARGET_HEIGHT_M
                - Constants.ShotConstants.SHOOTER_PIVOT_HEIGHT_M;

        final double dist = Math.max(distMeters, 0.05);

        // 1) Launch angle theta from desired entry angle phi
        final double phi = Constants.ShotConstants.DESIRED_PHI_RAD;
        final double theta = Math.atan((2.0 * h + dist * Math.tan(phi)) / dist);

        final double cosT = Math.cos(theta);

        // 2) Required exit velocity
        final double denom = 2.0 * cosT * cosT * (dist * Math.tan(theta) - h);
        if (denom <= 1e-6) {
            // Geometry not physically reachable (or bad constants)
            return new ShooterState(theta, 0.0, 0.0, 0.0);
        }

        final double vReq = Math.sqrt((g * dist * dist) / denom);

        // 3) Time of flight
        final double tof = dist / (vReq * cosT);

        // 4) Surface speed
        final double vSurface = vReq / Constants.ShotConstants.K_EFF;

        // 5) Convert surface speed to RPM for each wheel
        final double bottomRPM = (vSurface / Constants.ShotConstants.CIRC_BOTTOM_M) * 60.0;
        final double topRPM = (vSurface / Constants.ShotConstants.CIRC_TOP_M) * 60.0;

        return new ShooterState(theta, bottomRPM, topRPM, tof);
    }
}