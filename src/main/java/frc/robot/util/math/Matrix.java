package frc.robot.util.math;

/**
 * A comprehensive Matrix class for linear algebra operations.
 *
 * <p>This class provides functionalities such as matrix addition, subtraction,
 * multiplication, transposition, determinant, inverse, eigenvalue and eigenvector
 * calculations (for symmetric matrices using the Jacobi method), dot and cross products,
 * and more.
 *
 * <p><b>Usage Example:</b>
 * <pre>
 *     // Create a 3x3 matrix
 *     double[][] data = {
 *         { 4, 1, -2 },
 *         { 1, 2,  0 },
 *         { -2, 0,  3 }
 *     };
 *     Matrix m = new Matrix(data);
 *
 *     // Calculate the eigen decomposition (only works for symmetric matrices)
 *     Matrix.EigenDecomposition ed = m.eigenDecomposition();
 *     System.out.println("Eigenvalues: " + Arrays.toString(ed.eigenvalues));
 *     System.out.println("Eigenvectors: " + ed.eigenvectors);
 * </pre>
 * <hr>
 * @author Cameron Myhre
 * @version 2.1.0
 */
public class Matrix {

    private final int rows;
    private final int cols;
    private final double[][] data;

    /* ======================= Constructors ======================= */

    /**
     * Constructs a rows x cols matrix initialized with zeros.
     *
     * @param rows Number of rows.
     * @param cols Number of columns.
     */
    public Matrix(int rows, int cols) {
        this.rows = rows;
        this.cols = cols;
        this.data = new double[rows][cols];
    }

    /**
     * Constructs a matrix from a 2D array. A deep copy of the array is made.
     *
     * @param data 2D array containing the matrix data.
     */
    public Matrix(double[][] data) {
        this.rows = data.length;
        if (rows == 0) {
            throw new IllegalArgumentException("Matrix must have at least one row.");
        }
        this.cols = data[0].length;
        this.data = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            if (data[i].length != cols) {
                throw new IllegalArgumentException("All rows must have the same number of columns.");
            }
            System.arraycopy(data[i], 0, this.data[i], 0, cols);
        }
    }

    /**
     * Returns the number of rows in the matrix.
     *
     * @return Number of rows.
     */
    public int getRowCount() {
        return rows;
    }

    /**
     * Returns the number of columns in the matrix.
     *
     * @return Number of columns.
     */
    public int getColCount() {
        return cols;
    }

    /**
     * Returns the element at the specified row and column.
     *
     * @param row Row index (0-based).
     * @param col Column index (0-based).
     * @return Element value.
     */
    public double get(int row, int col) {
        return data[row][col];
    }

    /**
     * Sets the element at the specified row and column.
     *
     * @param row   Row index (0-based).
     * @param col   Column index (0-based).
     * @param value Value to set.
     */
    public void set(int row, int col, double value) {
        data[row][col] = value;
    }

    /* ======================= Basic Operations ======================= */

    /**
     * Adds the given matrix to this matrix.
     *
     * @param other The matrix to add.
     * @return A new Matrix representing the sum.
     * @throws IllegalArgumentException if dimensions do not match.
     */
    public Matrix add(Matrix other) {
        if (this.rows != other.rows || this.cols != other.cols) {
            throw new IllegalArgumentException("Matrix dimensions must agree for addition.");
        }
        Matrix result = new Matrix(rows, cols);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result.data[i][j] = this.data[i][j] + other.data[i][j];
            }
        }
        return result;
    }

    /**
     * Subtracts the given matrix from this matrix.
     *
     * @param other The matrix to subtract.
     * @return A new Matrix representing the difference.
     * @throws IllegalArgumentException if dimensions do not match.
     */
    public Matrix subtract(Matrix other) {
        if (this.rows != other.rows || this.cols != other.cols) {
            throw new IllegalArgumentException("Matrix dimensions must agree for subtraction.");
        }
        Matrix result = new Matrix(rows, cols);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result.data[i][j] = this.data[i][j] - other.data[i][j];
            }
        }
        return result;
    }

    /**
     * Multiplies this matrix by another matrix.
     *
     * @param other The matrix to multiply by.
     * @return A new Matrix representing the product.
     * @throws IllegalArgumentException if the inner dimensions do not match.
     */
    public Matrix multiply(Matrix other) {
        if (this.cols != other.rows) {
            throw new IllegalArgumentException("Matrix dimensions must agree for multiplication.");
        }
        Matrix result = new Matrix(this.rows, other.cols);
        for (int i = 0; i < this.rows; i++) {
            for (int j = 0; j < other.cols; j++) {
                for (int k = 0; k < this.cols; k++) {
                    result.data[i][j] += this.data[i][k] * other.data[k][j];
                }
            }
        }
        return result;
    }

    /**
     * Multiplies every element of the matrix by a scalar.
     *
     * @param scalar The scalar multiplier.
     * @return A new Matrix representing the scaled matrix.
     */
    public Matrix scale(double scalar) {
        Matrix result = new Matrix(rows, cols);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result.data[i][j] = this.data[i][j] * scalar;
            }
        }
        return result;
    }

    /**
     * Returns the transpose of the matrix.
     *
     * @return A new Matrix representing the transpose.
     */
    public Matrix transpose() {
        Matrix transposed = new Matrix(cols, rows);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                transposed.data[j][i] = this.data[i][j];
            }
        }
        return transposed;
    }

    /* ======================= Vector Operations ======================= */

    /**
     * Converts the matrix to a one-dimensional vector.
     * The matrix must be either a row vector (1 x n) or a column vector (n x 1).
     *
     * @return A double array representing the vector.
     * @throws IllegalArgumentException if the matrix is not a vector.
     */
    public double[] toVector() {
        if (rows == 1) {
            // Row vector
            double[] vec = new double[cols];
            System.arraycopy(this.data[0], 0, vec, 0, cols);
            return vec;
        } else if (cols == 1) {
            // Column vector
            double[] vec = new double[rows];
            for (int i = 0; i < rows; i++) {
                vec[i] = this.data[i][0];
            }
            return vec;
        } else {
            throw new IllegalArgumentException("Matrix is not a vector.");
        }
    }

    /**
     * Computes the dot product of two vectors represented as matrices.
     *
     * @param a First vector (row or column).
     * @param b Second vector (row or column).
     * @return The dot product.
     * @throws IllegalArgumentException if either matrix is not a vector or lengths differ.
     */
    public static double dot(Matrix a, Matrix b) {
        double[] vecA = a.toVector();
        double[] vecB = b.toVector();
        if (vecA.length != vecB.length) {
            throw new IllegalArgumentException("Vectors must have the same length for dot product.");
        }
        double sum = 0.0;
        for (int i = 0; i < vecA.length; i++) {
            sum += vecA[i] * vecB[i];
        }
        return sum;
    }

    /**
     * Computes the cross product of two 3-dimensional vectors.
     *
     * @param a First vector (must be 3-dimensional).
     * @param b Second vector (must be 3-dimensional).
     * @return A new Matrix representing the cross product as a column vector.
     * @throws IllegalArgumentException if either matrix is not a 3-dimensional vector.
     */
    public static Matrix cross(Matrix a, Matrix b) {
        double[] vecA = a.toVector();
        double[] vecB = b.toVector();
        if (vecA.length != 3 || vecB.length != 3) {
            throw new IllegalArgumentException("Cross product is only defined for 3-dimensional vectors.");
        }
        double[] result = new double[3];
        result[0] = vecA[1] * vecB[2] - vecA[2] * vecB[1];
        result[1] = vecA[2] * vecB[0] - vecA[0] * vecB[2];
        result[2] = vecA[0] * vecB[1] - vecA[1] * vecB[0];
        // Return as a column vector.
        double[][] resData = { { result[0] }, { result[1] }, { result[2] } };
        return new Matrix(resData);
    }

    /* ======================= Determinant & Inverse ======================= */

    /**
     * Returns a deep copy of the matrix.
     *
     * @return A new Matrix with the same data.
     */
    public Matrix copy() {
        return new Matrix(this.data);
    }

    /**
     * Swaps two rows of the matrix in place.
     *
     * @param i Index of the first row.
     * @param j Index of the second row.
     */
    private void swapRows(int i, int j) {
        double[] temp = data[i];
        data[i] = data[j];
        data[j] = temp;
    }

    /**
     * Computes the determinant of the matrix.
     *
     * @return The determinant.
     * @throws IllegalArgumentException if the matrix is not square.
     */
    public double determinant() {
        if (rows != cols) {
            throw new IllegalArgumentException("Determinant is only defined for square matrices.");
        }
        // Create a copy to perform elimination.
        Matrix copy = this.copy();
        double det = 1.0;
        for (int i = 0; i < rows; i++) {
            // Partial pivoting.
            int pivot = i;
            for (int j = i + 1; j < rows; j++) {
                if (Math.abs(copy.data[j][i]) > Math.abs(copy.data[pivot][i])) {
                    pivot = j;
                }
            }
            if (Math.abs(copy.data[pivot][i]) < 1e-10) {
                return 0;
            }
            if (i != pivot) {
                copy.swapRows(i, pivot);
                det = -det;
            }
            det *= copy.data[i][i];
            for (int j = i + 1; j < rows; j++) {
                double factor = copy.data[j][i] / copy.data[i][i];
                for (int k = i; k < cols; k++) {
                    copy.data[j][k] -= factor * copy.data[i][k];
                }
            }
        }
        return det;
    }

    /**
     * Returns the inverse of the matrix.
     *
     * @return A new Matrix representing the inverse.
     * @throws IllegalArgumentException if the matrix is not square.
     * @throws ArithmeticException      if the matrix is singular.
     */
    public Matrix inverse() {
        if (rows != cols) {
            throw new IllegalArgumentException("Only square matrices can be inverted.");
        }
        int n = rows;
        Matrix augmented = augmentWithIdentity();
        // Perform Gauss-Jordan elimination.
        for (int i = 0; i < n; i++) {
            // Find pivot.
            int pivot = i;
            for (int j = i; j < n; j++) {
                if (Math.abs(augmented.data[j][i]) > Math.abs(augmented.data[pivot][i])) {
                    pivot = j;
                }
            }
            if (Math.abs(augmented.data[pivot][i]) < 1e-10) {
                throw new ArithmeticException("Matrix is singular and cannot be inverted.");
            }
            augmented.swapRows(i, pivot);
            // Normalize the pivot row.
            double pivotVal = augmented.data[i][i];
            for (int j = 0; j < 2 * n; j++) {
                augmented.data[i][j] /= pivotVal;
            }
            // Eliminate the current column in other rows.
            for (int j = 0; j < n; j++) {
                if (j != i) {
                    double factor = augmented.data[j][i];
                    for (int k = 0; k < 2 * n; k++) {
                        augmented.data[j][k] -= factor * augmented.data[i][k];
                    }
                }
            }
        }
        // Extract the inverse matrix from the augmented matrix.
        double[][] invData = new double[n][n];
        for (int i = 0; i < n; i++) {
            System.arraycopy(augmented.data[i], n, invData[i], 0, n);
        }
        return new Matrix(invData);
    }

    /**
     * Augments the matrix with the identity matrix (useful for computing the inverse).
     *
     * @return A new Matrix of size n x 2n.
     */
    private Matrix augmentWithIdentity() {
        int n = rows;
        double[][] augmented = new double[n][2 * n];
        for (int i = 0; i < n; i++) {
            // Copy the original matrix.
            for (int j = 0; j < n; j++) {
                augmented[i][j] = this.data[i][j];
            }
            // Append the identity matrix.
            for (int j = n; j < 2 * n; j++) {
                augmented[i][j] = (i == (j - n)) ? 1.0 : 0.0;
            }
        }
        return new Matrix(augmented);
    }

    /* ======================= Eigen Decomposition ======================= */

    /**
     * Computes the eigenvalues and eigenvectors of a symmetric matrix using the Jacobi method.
     *
     * @return An EigenDecomposition object containing eigenvalues and eigenvectors.
     * @throws IllegalArgumentException if the matrix is not square or not symmetric.
     */
    public EigenDecomposition eigenDecomposition() {
        if (rows != cols) {
            throw new IllegalArgumentException("Eigen decomposition is only defined for square matrices.");
        }
        // Verify that the matrix is symmetric.
        for (int i = 0; i < rows; i++) {
            for (int j = i + 1; j < cols; j++) {
                if (Math.abs(this.data[i][j] - this.data[j][i]) > 1e-10) {
                    throw new IllegalArgumentException("Matrix is not symmetric.");
                }
            }
        }

        int n = rows;
        // Make a copy of the matrix for manipulation.
        double[][] a = new double[n][n];
        for (int i = 0; i < n; i++) {
            System.arraycopy(this.data[i], 0, a[i], 0, n);
        }
        // Initialize eigenvectors as the identity matrix.
        double[][] v = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                v[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }

        int maxIterations = 100;
        for (int iter = 0; iter < maxIterations; iter++) {
            // Find the largest off-diagonal element.
            int p = 0, q = 1;
            double maxOffDiag = Math.abs(a[p][q]);
            for (int i = 0; i < n; i++) {
                for (int j = i + 1; j < n; j++) {
                    if (Math.abs(a[i][j]) > maxOffDiag) {
                        maxOffDiag = Math.abs(a[i][j]);
                        p = i;
                        q = j;
                    }
                }
            }
            if (maxOffDiag < 1e-10) { // Convergence reached.
                break;
            }
            // Compute the Jacobi rotation.
            double phi = 0.5 * Math.atan2(2 * a[p][q], a[q][q] - a[p][p]);
            double c = Math.cos(phi);
            double s = Math.sin(phi);

            double app = c * c * a[p][p] - 2 * s * c * a[p][q] + s * s * a[q][q];
            double aqq = s * s * a[p][p] + 2 * s * c * a[p][q] + c * c * a[q][q];

            a[p][p] = app;
            a[q][q] = aqq;
            a[p][q] = 0;
            a[q][p] = 0;

            // Update the remaining elements.
            for (int i = 0; i < n; i++) {
                if (i != p && i != q) {
                    double aip = c * a[i][p] - s * a[i][q];
                    double aiq = s * a[i][p] + c * a[i][q];
                    a[i][p] = aip;
                    a[p][i] = aip;
                    a[i][q] = aiq;
                    a[q][i] = aiq;
                }
            }

            // Update the eigenvector matrix.
            for (int i = 0; i < n; i++) {
                double vip = c * v[i][p] - s * v[i][q];
                double viq = s * v[i][p] + c * v[i][q];
                v[i][p] = vip;
                v[i][q] = viq;
            }
        }

        // After convergence, the diagonal elements of 'a' are the eigenvalues.
        double[] eigenvalues = new double[n];
        for (int i = 0; i < n; i++) {
            eigenvalues[i] = a[i][i];
        }
        Matrix eigenvectors = new Matrix(v);
        return new EigenDecomposition(eigenvalues, eigenvectors);
    }

    /**
     * A container class for eigen decomposition results.
     */
    public static class EigenDecomposition {
        /**
         * The eigenvalues.
         */
        public final double[] eigenvalues;
        /**
         * The corresponding eigenvectors stored column-wise.
         */
        public final Matrix eigenvectors;

        /**
         * Constructs an EigenDecomposition with the given eigenvalues and eigenvectors.
         *
         * @param eigenvalues The array of eigenvalues.
         * @param eigenvectors The matrix whose columns are the corresponding eigenvectors.
         */
        public EigenDecomposition(double[] eigenvalues, Matrix eigenvectors) {
            this.eigenvalues = eigenvalues;
            this.eigenvectors = eigenvectors;
        }
    }

    /* ======================= Utility Methods ======================= */

    /**
     * Creates an identity matrix of the specified size.
     *
     * @param size The number of rows (and columns).
     * @return An identity Matrix.
     */
    public static Matrix identity(int size) {
        Matrix id = new Matrix(size, size);
        for (int i = 0; i < size; i++) {
            id.data[i][i] = 1.0;
        }
        return id;
    }

    /**
     * Returns a string representation of the matrix.
     *
     * @return A formatted string of the matrix.
     */
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        for (double[] row : data) {
            sb.append("[ ");
            for (double value : row) {
                sb.append(String.format("%10.4f ", value));
            }
            sb.append("]\n");
        }
        return sb.toString();
    }

    /* ======================= Main Method for Testing ======================= */

    public static void main(String[] args) {
        // Example: 3x3 symmetric matrix.
        double[][] data = {
            { 4, 1, -2 },
            { 1, 2,  0 },
            { -2, 0,  3 }
        };
        Matrix m = new Matrix(data);
        System.out.println("Matrix m:");
        System.out.println(m);

        // Compute determinant.
        System.out.println("Determinant: " + m.determinant());

        // Compute inverse.
        try {
            Matrix inv = m.inverse();
            System.out.println("Inverse of m:");
            System.out.println(inv);
        } catch (ArithmeticException ex) {
            System.out.println("Matrix is singular and cannot be inverted.");
        }

        // Compute eigen decomposition (Jacobi method).
        try {
            Matrix.EigenDecomposition ed = m.eigenDecomposition();
            System.out.println("Eigenvalues:");
            for (double lambda : ed.eigenvalues) {
                System.out.printf("%10.4f ", lambda);
            }
            System.out.println("\nEigenvectors (columns):");
            System.out.println(ed.eigenvectors);
        } catch (IllegalArgumentException ex) {
            System.out.println("Eigen decomposition error: " + ex.getMessage());
        }
    }
}
