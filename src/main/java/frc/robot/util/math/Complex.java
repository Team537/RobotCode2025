package frc.robot.util.math;

/**
 * The Complex class implements complex numbers and a variety of operations
 * including addition, subtraction, multiplication, division, exponentiation,
 * logarithms, nth roots, and trigonometric functions. It supports operations
 * between complex numbers as well as between complex and real numbers.
 *
 * <p>Example usage:
 * <pre>
 *     Complex z1 = new Complex(3, 4);   // 3 + 4i
 *     Complex z2 = new Complex(1, -2);  // 1 - 2i
 *
 *     Complex sum = z1.add(z2);         // Addition: (3+1) + (4-2)i = 4 + 2i
 *     Complex prod = z1.multiply(z2);   // Multiplication
 *     Complex div = z1.divide(z2);      // Division
 *
 *     double magnitude = z1.abs();      // |z1| = 5
 *     double angle = z1.arg();          // Argument in radians
 *
 *     Complex expZ1 = z1.exp();         // e^(3+4i)
 *     Complex logZ1 = z1.log();         // ln(3+4i)
 *
 *     // Raise to a real power:
 *     Complex z1Pow2 = z1.pow(2);
 *
 *     // Raise to a complex power:
 *     Complex z1PowZ2 = z1.pow(z2);
 *
 *     // Compute the 3rd roots of z1:
 *     Complex[] roots = z1.nthRoots(3);
 *
 *     // Trigonometric functions:
 *     Complex sinZ1 = z1.sin();
 *     Complex cosZ1 = z1.cos();
 *     Complex tanZ1 = z1.tan();
 * </pre>
 * <hr>
 * @author Cameron Myhre
 * @version 3.0.0
 */
public class Complex {

    // Real and imaginary parts of the complex number.
    private final double re;
    private final double im;

    /**
     * Constructs a complex number with the specified real and imaginary parts.
     *
     * @param re The real part.
     * @param im The imaginary part.
     */
    public Complex(double re, double im) {
        this.re = re;
        this.im = im;
    }

    /**
     * Returns the real part.
     *
     * @return The real part.
     */
    public double getReal() {
        return re;
    }

    /**
     * Returns the imaginary part.
     *
     * @return The imaginary part.
     */
    public double getImaginary() {
        return im;
    }

    /* ================= Basic Arithmetic Operations ================= */

    /**
     * Adds this complex number to another.
     *
     * @param other The other complex number.
     * @return A new Complex number representing the sum.
     */
    public Complex add(Complex other) {
        return new Complex(this.re + other.re, this.im + other.im);
    }

    /**
     * Adds a real number to this complex number.
     *
     * @param d The real number.
     * @return A new Complex number representing the sum.
     */
    public Complex add(double d) {
        return new Complex(this.re + d, this.im);
    }

    /**
     * Subtracts another complex number from this one.
     *
     * @param other The other complex number.
     * @return A new Complex number representing the difference.
     */
    public Complex subtract(Complex other) {
        return new Complex(this.re - other.re, this.im - other.im);
    }

    /**
     * Subtracts a real number from this complex number.
     *
     * @param d The real number.
     * @return A new Complex number representing the difference.
     */
    public Complex subtract(double d) {
        return new Complex(this.re - d, this.im);
    }

    /**
     * Multiplies this complex number by another.
     *
     * @param other The other complex number.
     * @return A new Complex number representing the product.
     */
    public Complex multiply(Complex other) {
        return new Complex(
            this.re * other.re - this.im * other.im,
            this.re * other.im + this.im * other.re
        );
    }

    /**
     * Multiplies this complex number by a real number.
     *
     * @param d The real number.
     * @return A new Complex number representing the product.
     */
    public Complex multiply(double d) {
        return new Complex(this.re * d, this.im * d);
    }

    /**
     * Divides this complex number by another.
     *
     * @param other The divisor complex number.
     * @return A new Complex number representing the quotient.
     * @throws ArithmeticException if the divisor is zero.
     */
    public Complex divide(Complex other) {
        double denominator = other.re * other.re + other.im * other.im;
        if (denominator == 0) {
            throw new ArithmeticException("Division by zero.");
        }
        return new Complex(
            (this.re * other.re + this.im * other.im) / denominator,
            (this.im * other.re - this.re * other.im) / denominator
        );
    }

    /**
     * Divides this complex number by a real number.
     *
     * @param d The real number.
     * @return A new Complex number representing the quotient.
     * @throws ArithmeticException if d is zero.
     */
    public Complex divide(double d) {
        if (d == 0) {
            throw new ArithmeticException("Division by zero.");
        }
        return new Complex(this.re / d, this.im / d);
    }

    /**
     * Returns the conjugate of this complex number.
     *
     * @return A new Complex number representing the conjugate.
     */
    public Complex conjugate() {
        return new Complex(this.re, -this.im);
    }

    /* ================= Advanced Operations ================= */

    /**
     * Returns the absolute value (magnitude) of this complex number.
     *
     * @return The magnitude.
     */
    public double abs() {
        return Math.hypot(this.re, this.im);
    }

    /**
     * Returns the argument (phase angle in radians) of this complex number.
     *
     * @return The argument in radians.
     */
    public double arg() {
        return Math.atan2(this.im, this.re);
    }

    /**
     * Creates a complex number from polar coordinates.
     *
     * @param r     The magnitude.
     * @param theta The angle in radians.
     * @return A new Complex number corresponding to (r, theta).
     */
    public static Complex fromPolar(double r, double theta) {
        return new Complex(r * Math.cos(theta), r * Math.sin(theta));
    }

    /**
     * Computes the exponential of this complex number.
     *
     * <p>exp(z) = e^(x) * (cos(y) + i sin(y)), where z = x + iy.
     *
     * @return A new Complex number representing e^(z).
     */
    public Complex exp() {
        double expRe = Math.exp(this.re);
        return new Complex(expRe * Math.cos(this.im), expRe * Math.sin(this.im));
    }

    /**
     * Computes the natural logarithm of this complex number (principal value).
     *
     * <p>ln(z) = ln(|z|) + i arg(z)
     *
     * @return A new Complex number representing the logarithm.
     */
    public Complex log() {
        return new Complex(Math.log(this.abs()), this.arg());
    }

    /**
     * Raises this complex number to a real power.
     *
     * <p>z^n = exp(n * ln(z))
     *
     * @param n The real exponent.
     * @return A new Complex number representing z^n.
     */
    public Complex pow(double n) {
        return this.log().multiply(n).exp();
    }

    /**
     * Raises this complex number to a complex power.
     *
     * <p>z^w = exp(w * ln(z))
     *
     * @param exponent The complex exponent.
     * @return A new Complex number representing z^w.
     */
    public Complex pow(Complex exponent) {
        return this.log().multiply(exponent).exp();
    }

    /**
     * Returns the principal square root of this complex number.
     *
     * @return A new Complex number representing the principal square root.
     */
    public Complex sqrt() {
        return nthRoots(2)[0];
    }

    /**
     * Computes all the nth roots of this complex number.
     *
     * <p>If z = r * exp(iθ), then its n distinct nth roots are given by:
     * <pre>
     *   root_k = r^(1/n) * exp(i*(θ + 2πk)/n),  k = 0, 1, ..., n-1
     * </pre>
     *
     * @param n The degree of the root (n > 0).
     * @return An array of Complex numbers representing all the nth roots.
     * @throws IllegalArgumentException if n is not positive.
     */
    public Complex[] nthRoots(int n) {
        if (n <= 0) {
            throw new IllegalArgumentException("n must be a positive integer.");
        }
        double r = this.abs();
        double theta = this.arg();
        double rootMag = Math.pow(r, 1.0 / n);
        Complex[] roots = new Complex[n];
        for (int k = 0; k < n; k++) {
            double angle = (theta + 2 * Math.PI * k) / n;
            roots[k] = Complex.fromPolar(rootMag, angle);
        }
        return roots;
    }

    /* ================= Trigonometric and Hyperbolic Functions ================= */

    /**
     * Computes the sine of this complex number.
     *
     * <p>sin(z) = sin(x) cosh(y) + i cos(x) sinh(y)
     *
     * @return A new Complex number representing sin(z).
     */
    public Complex sin() {
        return new Complex(
            Math.sin(this.re) * Math.cosh(this.im),
            Math.cos(this.re) * Math.sinh(this.im)
        );
    }

    /**
     * Computes the cosine of this complex number.
     *
     * <p>cos(z) = cos(x) cosh(y) - i sin(x) sinh(y)
     *
     * @return A new Complex number representing cos(z).
     */
    public Complex cos() {
        return new Complex(
            Math.cos(this.re) * Math.cosh(this.im),
            -Math.sin(this.re) * Math.sinh(this.im)
        );
    }

    /**
     * Computes the tangent of this complex number.
     *
     * <p>tan(z) = sin(z) / cos(z)
     *
     * @return A new Complex number representing tan(z).
     */
    public Complex tan() {
        return this.sin().divide(this.cos());
    }

    /* ================= Utility Methods ================= */

    /**
     * Returns a string representation of this complex number.
     *
     * @return A string in the form "a + bi" or "a - bi".
     */
    @Override
    public String toString() {
        if (Math.abs(im) < 1e-10) {
            return String.format("%.4f", re);
        }
        if (Math.abs(re) < 1e-10) {
            return String.format("%.4fi", im);
        }
        if (im < 0) {
            return String.format("%.4f - %.4fi", re, -im);
        } else {
            return String.format("%.4f + %.4fi", re, im);
        }
    }

    /**
     * Checks for equality between this complex number and another object.
     *
     * @param obj The other object.
     * @return true if obj is a Complex number with nearly equal real and imaginary parts.
     */
    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof Complex)) return false;
        Complex other = (Complex) obj;
        return Math.abs(this.re - other.re) < 1e-10 &&
               Math.abs(this.im - other.im) < 1e-10;
    }

    /**
     * Returns a hash code for this complex number.
     *
     * @return The hash code.
     */
    @Override
    public int hashCode() {
        return Double.hashCode(re) * 31 + Double.hashCode(im);
    }
}