package uk.ac.nottingham.cryptography.galois;

import java.util.Arrays;

/**
 * Unfinished implementation of a multiplier in GF(2^128).
 * <p>
 * This class could be used to implement a more efficient
 * multiplier in GF(2^128). Coding in this class is not
 * necessary unless you wish to tackle this challenge.
 * <p>
 * This class is used by the FastGFTests class within
 * the test suite, which is disabled by default.
 */
public class GF128FastImpl implements GF128Multiplier {

    private static final byte[] R = new byte[] { (byte)0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    private byte[] H = null;
    private byte[] M = null;

    // Pseudocode given from Algorithm 3 on the specification
    @Override
    public void init(byte[] H) {
        this.H = H;
        this.M = H;
        int i = 8;
        byte[] P = new byte[16];
        P[i] = 1;
        while(i > 0) {
            M[i] = (byte)(M[2*i] * P[i]);
            i=i/2;
        }
        i=2;
        while(i < 128) {
            for(int j = 1; j < i-1; j++) {
                M[i+j] = xorBytes(M[i], M[j]);
            }
            i=2*i;
        }
        Arrays.fill(M, (byte) 0);
    }

    // This method XOR two bytes together
    public static byte xorBytes(byte byte1, byte byte2) {
        return (byte) (byte1 ^ byte2);
    }

    @Override
    public void multiplyByH(byte[] X) {
        if (H != null) {
            multiply(X, H);
        }
    }

    // Pseudocode given from Algorithm 2 on the specification
    @Override
    public void multiply(byte[] X, byte[] Y) {
        byte[] Z = new byte[16];
        for(int i = 15; i > 0; i--) {
            Z[i] = (byte) (Z[i] ^ getBit(X, i));
            byte A = (byte) getBit(X, 15);
            for(int j = 15; j > 1; j--) {
                byte Xj = (byte) getBit(X, j);
                X[j] = X[Xj];
                X[j] = (byte) getBit(X, j-1);
            }
            Z[i] ^= R[A];
        }
    }

    @Override
    public byte[] getH() {
        return this.H;
    }

    private int getBit(byte[] a, int i) {
        return (a[15 - i / 8] >> (i % 8)) & 1;
    }
}
