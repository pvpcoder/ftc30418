package org.firstinspires.ftc.teamcode.turret;

/**
 * A short timestamped history of the robot's pose.
 *
 * WHY THIS EXISTS
 * ---------------
 * A camera frame is always a little old by the time we can act on it. To turn
 * "the tag was 40 inches ahead and 12 to the left of the lens" into a FIELD
 * position, we need the robot's pose AT THE MOMENT THE SHUTTER FIRED. Using
 * the current pose instead smears the target by however far the robot moved
 * in the meantime, which on a driving robot is inches.
 *
 * Pedro's own PoseHistory does not help: it stores bare X and Y arrays for
 * drawing on the dashboard, with no timestamps and no heading. So we keep our
 * own record.
 *
 * This is NOT a second localizer. It records the output of the one Follower
 * the OpMode already owns, sampled once per loop in that loop. Nothing here
 * calls update() on anything.
 */
public class PoseBuffer {

    private static final int CAPACITY = 128;

    private final long[]   nanos = new long[CAPACITY];
    private final double[] xs    = new double[CAPACITY];
    private final double[] ys    = new double[CAPACITY];
    private final double[] hs    = new double[CAPACITY];

    private int count = 0, head = 0;
    private final long maxAgeNanos;

    public PoseBuffer(double maxAgeMs) {
        this.maxAgeNanos = (long) (maxAgeMs * 1e6);
    }

    /** Call once per loop with the pose the Follower just reported. */
    public void record(long t, double x, double y, double headingRad) {
        nanos[head] = t;
        xs[head] = x;
        ys[head] = y;
        hs[head] = headingRad;
        head = (head + 1) % CAPACITY;
        if (count < CAPACITY) count++;
    }

    public void clear() { count = 0; head = 0; }

    public boolean isEmpty() { return count == 0; }

    /** A pose with the time it was taken. */
    public static final class Sample {
        public final long t;
        public final double x, y, heading;
        public final long ageNanos;
        Sample(long t, double x, double y, double heading, long ageNanos) {
            this.t = t; this.x = x; this.y = y; this.heading = heading;
            this.ageNanos = ageNanos;
        }
        public Geometry.Vec pos() { return new Geometry.Vec(x, y); }
    }

    /**
     * The recorded pose closest in time to `targetNanos`.
     *
     * Returns null if we have nothing within maxAge of that instant, rather
     * than handing back the nearest thing we happen to have. A caller that
     * gets null must refuse the observation. Quietly substituting a stale
     * pose is exactly the kind of fabrication that produces an aim that looks
     * plausible and is wrong.
     */
    public Sample nearest(long targetNanos) {
        if (count == 0) return null;

        long bestDiff = Long.MAX_VALUE;
        int bestIdx = -1;
        for (int i = 0; i < count; i++) {
            int idx = (head - 1 - i + CAPACITY * 2) % CAPACITY;
            long d = Math.abs(nanos[idx] - targetNanos);
            if (d < bestDiff) { bestDiff = d; bestIdx = idx; }
        }

        if (bestIdx < 0 || bestDiff > maxAgeNanos) return null;
        return new Sample(nanos[bestIdx], xs[bestIdx], ys[bestIdx], hs[bestIdx], bestDiff);
    }
}
