package ballistics;

public class Vector {
    public final double x;
    public final double y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    
    public Vector subtract(Vector vec) {
        return new Vector(this.x - vec.x, this.y - vec.y);
    }

    public Vector add(Vector vec) {
        return new Vector(this.x + vec.x, this.y + vec.y);
    }

    public double norm() {
        return Math.hypot(this.x, this.y);
    }

    public double sqrNorm() {
        return this.x * this.x + this.y * this.y;
    }

    public double arctan() {
        return Math.atan2(this.y, this.x);
    }

    public double dot(Vector vec) {
        return this.x * vec.x + this.y * vec.y;
    }

    // public Vector transform(Matrix matrix) {
    //     return new Vector(
    //         matrix.a * this.x + matrix.c * this.y + matrix.e,
    //         matrix.b * this.x + matrix.d * this.y + matrix.f
    //     );
    // }

    public Vector scale(double scalar) {
        return new Vector(scalar * this.x, scalar * this.y);
    }

    public String toString() {
        return "Vector<" + this.x + ", " + this.y + ">";
    }

}