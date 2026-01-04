package org.firstinspires.ftc.teamcode.ballistics;

import com.pedropathing.math.Vector;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.util.LinkedList;

public final class BallisticFileIo {
    public static final int METADATA_BYTES = Float.BYTES + Integer.BYTES;

    /**
     * Writes the arcs's points and start velocities to the file.
     */
    public static File appendArc(File file, BallisticArc arc) throws IOException {
        final FileOutputStream writer = new FileOutputStream(file, true);
        try {
            appendArc(writer, arc);
        } catch(IOException exception) {
            writer.close();
            throw exception;
        }

        writer.close();
        return file;
    }

    /**
     * Writes the arcs's points and start velocities using the FileOutputStream. The writer is 
     * not closed by this method.
     */
    public static void appendArc(FileOutputStream writer, BallisticArc arc) throws IOException {
        // Writing the metadata of the points
        writeFloats(writer, (float) arc.getElapsedTime());
        writeInt(writer, arc.size());

        try {
            // Writing the points themselves
            arc.pointIterator().forEachRemaining((point) -> {
                try {
                    writeFloats(writer, (float) point.getXComponent(), (float) point.getYComponent());
                } catch(IOException exception) {
                    throw new InnerException(exception);
                }
            });

            // Writing the velocities
            arc.velIterator().forEachRemaining((vel) -> {
                try {
                    writeFloats(writer, (float) vel.getXComponent(), (float) vel.getYComponent());
                } catch(IOException exception) {
                    throw new InnerException(exception);
                }
            });
        } catch(InnerException exception) {
            throw ((IOException) exception.getCause());
        }
    }

    /**
     * An arc that only stores its first and last of the points and velocities.
     */
    private static final class FirstLastBallisticArc implements BallisticArc {
        private final double elapsed;

        private final Vector firstPoint;
        private final Vector lastPoint;

        private final Vector firstVelocity;
        private final Vector lastVelocity;

        FirstLastBallisticArc(BallisticArc arc) {
            this.elapsed = arc.getElapsedTime();

            this.firstPoint = arc.firstPoint();
            this.lastPoint = arc.lastPoint();
            this.firstVelocity = arc.firstVel();
            this.lastVelocity = arc.lastVel();
        }

        @Override
        public Vector getPoint(int i) {
            if(i == 0) {
                return this.firstPoint;
            }

            if(i == 1) {
                return this.lastPoint;
            }

            // Out of bounds
            throw new IndexOutOfBoundsException(i);
        }

        @Override
        public Vector getVel(int i) {
            if(i == 0) {
                return this.firstVelocity;
            }

            if(i == 1) {
                return this.lastVelocity;
            }

            // Out of bounds
            throw new IndexOutOfBoundsException(i);
        }

        @Override
        public int size() {
            return 2;
        }
    
        @Override
        public double getElapsedTime() {
            return this.elapsed;
        }
    
        @Override
        public boolean compute(double unused1, ArcFunction unused2) {
            return false;
        }
    }

    /**
     * Supplies data for a ballistic arc lazily. This is in contrast to ComputableBallisiticArc,
     * which stores all its data at once upon computation. These, however, cannot be computed
     */
    private static final class BallisticArcStream implements BallisticArc {
        private final InputStream stream; 
        private final double elapsed;
        private final int size;

        /**
         * Creates a ballistic arc that sources its data from a file.
         * @param stream The file data. The position must be at the first byte of data 
         * (immediately after the size metadata field)
         * @param elapsed The time surmised by the arc. Returned by `getElapsedTime()`
         * @param pointSize The number of points in the arc. This is equal to the number of 
         * velocities but **not** the number of bytes. 
         */
        BallisticArcStream(InputStream stream, float elapsed, int pointSize) throws IOException {
            this.elapsed = elapsed;
            this.size = pointSize;
            this.stream = stream;
        }

        @Override
        public int size() {
            return this.size;
        }

        @Override
        public Vector getPoint(int i) {
            try {
                final byte[] buf = new byte[2 * Float.BYTES];
                stream.reset();
                stream.skip(buf.length * i);
                stream.read(buf);
                return new Vector(
                    Float.intBitsToFloat(intFromBuf(buf, 0)), 
                    Float.intBitsToFloat(intFromBuf(buf, 4))
                ); 
            } catch(IOException exception) {
                throw new InnerException(exception);
            }
        }

        @Override
        public Vector getVel(int i) {
            try {
                final byte[] buf = new byte[2 * Float.BYTES];
                stream.reset();
                stream.skip(buf.length * (i + size));
                stream.read(buf);
                return new Vector(
                    Float.intBitsToFloat(intFromBuf(buf, 0)), 
                    Float.intBitsToFloat(intFromBuf(buf, 4))
                ); 
            } catch(IOException exception) {
                throw new InnerException(exception);
            }
        }

        @Override
        public double getElapsedTime() {
            return this.elapsed;
        }

        @Override
        public boolean compute(double unused1, ArcFunction unused2) {
            return false;
        }

        /**
         * Creates an `ImmutableBallisticArc` that only has the correct time, 
         * first point, last point, first velocity, and last velocity. No 
         * in-between points are kept 
         * 
         * @return An ImmutableBallisticArc with only the important aspects
         */
        public BallisticArc concretize() {
            return new ImmutableBallisticArc(new FirstLastBallisticArc(this));
        }
    }

    private static int unsigned(byte bits) {
        return ((int) bits) << 24 >>> 24;
    }

    private static int intFromBuf(byte[] buf, int start) {
        int result = 0;
        for(int i = 0; i < Integer.BYTES; i++) {
            result = result | (unsigned(buf[start + i]) << (i << 3));
        }
        return result;
    }

    /**
     * Returns all the arcs in the file that satisfy the given condition. 
     */
    public static LinkedList<BallisticArc> readArcs(File file, ArcFunction filter) throws IOException {
        LinkedList<BallisticArc> result = null;
        final FileInputStream reader = new FileInputStream(file);

        try {
            result = readArcs(reader, filter);
        } catch(IOException | InnerException exception) {
            reader.close();
            throw exception;
        }

        reader.close();
        return result;
    }

    public static LinkedList<BallisticArc> readArcs(InputStream reader, ArcFunction filter) throws IOException {
        final LinkedList<BallisticArc> result = new LinkedList<>();
        int position = 0;

        try {
            final int MAX_ITERS = 100_000; // Infinite loop prevention 

            // NOTE: This loop is ordinarily broken by a break statement inside the loop
            //       The only reason we are using a for loop is to provide infinite loop 
            //       prevention
            fileParsingLoop:
            for(int iters = 0; iters < MAX_ITERS; iters++) {
                // Gettting the meta data
                final byte[] buf = new byte[METADATA_BYTES];
                position += buf.length;

                if(reader.read(buf) == -1) {
                    // read only returns -1 if the end of the file has been reached.
                    break fileParsingLoop;
                }
                
                final float elapsed = Float.intBitsToFloat(intFromBuf(buf, 0));
                final int size = intFromBuf(buf, 4); // in point vectors, not total point and velocit vecs nor file bytes

                // Creating the ballistic arc stream
                final int dataByteLength = size * 4 * Float.BYTES;
                reader.mark(dataByteLength);
                final BallisticArcStream stream = new BallisticArcStream(reader, elapsed, size);

                // Concretizing the arc and adding it to the result only if it satisfies the filter
                if(filter.apply(stream)) {
                    result.add(stream.concretize());
                }
                
                // Going to the next arc
                reader.reset();
                reader.skip(dataByteLength);
                position += dataByteLength;
            }
        } catch(IOException | InnerException exception) {
            throw exception;
        }

        return result;
    }

    private static void writeFloats(FileOutputStream writer, float... floats) throws IOException {
        // Serializing into a byte buffer
        // Floats are stored little endian
        final byte[] buf = new byte[floats.length * Float.BYTES];

        for(int bufStartIndex = 0; bufStartIndex < buf.length; bufStartIndex += 4) {
            final int bits = Float.floatToIntBits(floats[bufStartIndex >> 2]);
            for(int i = 0; i < Float.BYTES; i++) {
                final int shifter = i << 3; // 8 * i
                buf[bufStartIndex + i] = (byte) ((bits & (0xff << shifter)) >>> shifter);

            }
        }

        // Writing the bytes
        writer.write(buf);
    }

    private static void writeInt(FileOutputStream writer, int bits) throws IOException {
        // Serializing into a byte buffer
        // Floats are stored little endian
        final byte[] buf = new byte[Integer.BYTES];
        for(int i = 0; i < Integer.BYTES; i++) {
            final int shifter = i << 3; // 8 * i
            buf[i] = (byte) ((bits & (0xff << shifter)) >>> shifter);
        }

        // Writing the bytes
        writer.write(buf);
    }
}