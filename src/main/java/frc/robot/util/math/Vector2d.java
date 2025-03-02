package frc.robot.util.math;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * <h2> Vector2d </h2>
 * The {@code Vector2d} class is a class that represents a 2 dimensional vector. It is used extensively in the 
 * {@code DriveSubsystem}, where it allows for certain trajectories to be reliably and efficiently targeted.
 * <hr>
 * @author Parker Huibregtse
 * @since v1.1.0
 */
public class Vector2d {

    // Components of the vector
    private double x;
    private double y;

    /**
     * Creates a vector object
     * 
     * @param x the x component of the vector
     * @param y the y component of the vector
     */
    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Gets the x component of the vector
     * 
     * @return the x component
     */
    public double getX() {
        return x;
    }

    /**
     * Gets the y component of the vector
     * 
     * @return the y component
     */
    public double getY() {
        return y;
    }

    /**
     * Sets the x component of the vector
     * 
     * @param x the new x component
     */
    public void setX(double x) {
        this.x = x;
    }

    /**
     * Sets the y component of the vector
     * 
     * @param y the new y component
     */
    public void setY(double y) {
        this.y = y;
    }

    /**
     * Computes the magnitude (length) of the vector
     * 
     * @return the magnitude of the vector
     */
    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Normalizes the vector to have a magnitude of 1
     * 
     * @return a new normalized vector
     */
    public Vector2d normalize() {
        double magnitude = magnitude();
        if (magnitude == 0) {
            return new Vector2d(0, 0);
        }
        return new Vector2d(x / magnitude, y / magnitude);
    }

    /**
     * Computes the dot product with another vector
     * 
     * @param other the other vector
     * @return the dot product
     */
    public double dot(Vector2d other) {
        return this.x * other.x + this.y * other.y;
    }

    /**
     * Computes the cross product (scalar) with another vector
     * 
     * @param other the other vector
     * @return the scalar cross product
     */
    public double cross(Vector2d other) {
        return this.x * other.y - this.y * other.x;
    }

    /**
     * Rotates the vector by a given angle using a Rotation2d object
     * 
     * @param rotation the Rotation2d object
     * @return a new rotated vector
     */
    public Vector2d rotateBy(Rotation2d rotation) {
        double cos = Math.cos(rotation.getRadians());
        double sin = Math.sin(rotation.getRadians());
        return new Vector2d(x * cos - y * sin, x * sin + y * cos);
    }

    /**
     * Adds this vector to another vector
     * 
     * @param other the other vector
     * @return a new vector that is the sum of the two
     */
    public Vector2d add(Vector2d other) {
        return new Vector2d(this.x + other.x, this.y + other.y);
    }

    /**
     * Subtracts another vector from this vector
     * 
     * @param other the other vector
     * @return a new vector that is the difference of the two
     */
    public Vector2d subtract(Vector2d other) {
        return new Vector2d(this.x - other.x, this.y - other.y);
    }

    /**
     * Scales the vector by a scalar value
     * 
     * @param scalar the scalar to scale by
     * @return a new scaled vector
     */
    public Vector2d scale(double scalar) {
        return new Vector2d(this.x * scalar, this.y * scalar);
    }

    /**
     * Computes the angle of this vector
     * 
     * @return the angle
     */
    public Rotation2d angle() {
        return new Rotation2d(Math.atan2(y, x));
    }

    /**
     * Projects this vector onto another vector
     * 
     * @param other the vector to project onto
     * @return the projected vector
     */
    public Vector2d projectOnto(Vector2d other) {
        double scalar = this.dot(other) / other.magnitude();
        return other.normalize().scale(scalar);
    }

    /**
     * Computes the distance to another vector
     * 
     * @param other the other vector
     * @return the distance between the two vectors
     */
    public double distanceTo(Vector2d other) {
        return Math.sqrt(Math.pow(this.x - other.x, 2) + Math.pow(this.y - other.y, 2));
    }

    /**
     * Checks if this vector is equal to another vector
     * 
     * @param other the other vector
     * @return true if the vectors are equal, false otherwise
     */
    @Override
    public boolean equals(Object other) {
        if (!(other instanceof Vector2d)) {
            return false;
        }
        Vector2d otherVector = (Vector2d) other;
        return Double.compare(this.x, otherVector.x) == 0 && Double.compare(this.y, otherVector.y) == 0;
    }

    /**
     * Override the hashCode method to follow best practices.
     */
    @Override
    public int hashCode() {
        // Use a common algorithm for doubles
        int result = 17;
        long xBits = Double.doubleToLongBits(x);
        long yBits = Double.doubleToLongBits(y);
        result = 31 * result + (int)(xBits ^ (xBits >>> 32));
        result = 31 * result + (int)(yBits ^ (yBits >>> 32));
        return result;
    }
    
    /**
     * Provides a string representation of the vector
     * 
     * @return a string in the format (x, y)
     */
    @Override
    public String toString() {
        return String.format("(%.2f, %.2f)", x, y);
    }
}