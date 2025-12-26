package ballistics;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

public class Main {
    public static void main(String[] args) throws IOException {
        final File pre = new File("./arcs-0.5deg-5.0in-pre-apex.bin");
        final File post = new File("./arcs-0.5deg-5.0in-post-apex.bin");
        final File combined = new File("./arcs-0.5deg-0.5in-combined.bin");

        final FileOutputStream output = new FileOutputStream(combined);

        // NOTE: This deletes the combined file
        if(combined.exists()) {
            combined.delete();
        }

        if(!combined.exists()) {
            combined.createNewFile();
        }

        // Reading pre
        System.out.println("Reading the pre arcs...");
        final BallisticArcSelection preArcs = new BallisticArcSelection(
            BallisticFileIo.readArcs(pre, (unused) -> true)
        );
        System.out.println("Reading the post arcs...");
        final BallisticArcSelection postArcs = new BallisticArcSelection(
            BallisticFileIo.readArcs(post, (unused) -> true)
        );

        System.out.println("Writing the pre arcs...");
        for(final BallisticArc preArc : preArcs.toArray()) {
            BallisticFileIo.appendArc(output, preArc);
        }

        System.out.println("Writing the post arcs...");
        for(final BallisticArc postArc : postArcs.toArray()) {
            BallisticFileIo.appendArc(output, postArc);
        }

        output.close();
    }

    public static void computePreApex(String[] args) throws IOException {
        final File outputFile = new File("./arcs-0.5deg-5.0in-pre-apex.bin");

        if(!outputFile.exists()) {
            System.out.println("Could not find file");
            return;
        }

        final FileOutputStream outputWriter = new FileOutputStream(outputFile);

        // Getting the constants of computation.
        final double SCALAR = 0.0197 * 157.1 * 0.90 / 74.842741; // in^-1
        final Vector GRAVITY = new Vector(0, -386.08); // in / s^2
        final double TARGET_HEIGHT = 39; // in
        
        // What determines whether we keep going or not
        // This ends if we go below the ground or we pass through the goal
        final ArcFunction scored = (a) -> 
            a.getPoint(a.size() - 2).y <= TARGET_HEIGHT 
            && a.getPoint(a.size() - 1).y > TARGET_HEIGHT;
        final ArcFunction notAtHeightOrFloor = (a) -> 
            a.getCurrentPoint().y >= 0
            && !scored.apply(a);
        
        // Constants that determine our input.
        final double STEP_SIZE = 1.0 / 480.0; // sec
        final double THETA_STEP = Math.toRadians(0.5);
        final double SPEED_STEP = 5; // in/s
        
        
        // Initializaing the benchmarks.
        int iterations = 0;
        int added =0;
        final long startTime = System.nanoTime();

        for(double theta = Math.toRadians(45); theta < Math.toRadians(70); theta += THETA_STEP) {
            System.out.printf(
                "[%.2f sec] Computing %.2f°...\n",
                (System.nanoTime() - startTime) / Math.pow(10, 9), 
                Math.toDegrees(theta)
            );

            for(double speed = 150; speed < 400; speed += SPEED_STEP) {
                final BallisticArc arc = new ComputableBallisiticArc.Builder()
                    .setDragScalar(SCALAR)
                    .setGravity(GRAVITY)
                    .setVel(theta, speed)
                    .build();

                arc.compute(STEP_SIZE, notAtHeightOrFloor);

                // Making sure that it actually scored, not hit the floor
                if(scored.apply(arc)) {
                    // It did, so do the thing
                    added++;
                    BallisticFileIo.appendArc(outputWriter, arc);

                    if(added % 250 == 0 && added != 0) {
                        System.out.printf(
                            "[%.2f sec] Added total of %d\n",
                            (System.nanoTime() - startTime) / Math.pow(10, 9), 
                            added
                        );
                    }
                }

                iterations++;
            }
        }

        outputWriter.close();

        final double elapsed = (System.nanoTime() - startTime) / Math.pow(10, 9);
        System.out.println("Computed Arcs: " + iterations);
        System.out.printf("Total Runtime: %.2f sec\n", elapsed);
        System.out.printf("Avg Runtime: %.3f ms\n", 1_000 * elapsed / iterations);
    }
}