package uk.ac.nottingham.cryptography;

import uk.ac.nottingham.cryptography.aes.AES128Encryptor;
import uk.ac.nottingham.cryptography.aes.AES128EncryptorImpl;
import uk.ac.nottingham.cryptography.galois.GF128Multiplier;
import uk.ac.nottingham.cryptography.galois.GF128MultiplierImpl;
import java.util.Arrays;

/**
 * Implementation of AEADCipher that encrypts using AES and calculates
 * a tag using GCM.
 * <p> 
 * This class is the primary code file in which you can complete your
 * solution to the coursework.
 */
public class AESGCM implements AEADCipher {

    private final GF128Multiplier GF;
    private final AES128Encryptor encryptor;
    private CipherMode mode;
    byte[] iv, key, counter, tag, mask;
    long lengthC, lengthA;

    public AESGCM() {
        GF = new GF128MultiplierImpl();
        encryptor = new AES128EncryptorImpl();
    }

    // This method initialises the cipher using the key, IV, and CipherMode
    @Override
    public void init(AEADParams params) {
        iv = params.getIv(); // initialising the iv
        key = params.getKey(); // initialising the key
        mode = params.getMode(); // initialising the CipherMode

        tag = new byte[16]; // creating an empty tag
        lengthA = 0; // AAD length initialisation
        lengthC = 0; // Ciphertext length initialisation

        // encryption done by passing the key
        encryptor.init(key);

        // encrypt an all zero block
        byte[] zeroBlock = new byte[16];
        encryptor.encryptBlock(new byte[16], zeroBlock);
        // initialise the hash key in GF
        GF.init(zeroBlock);

        // this method checks the length of the iv
        // and applies the correct functionality depending on iv length
        // such as the GHASH function
        counter = new byte[16];
        if(iv.length == 12) {
            counter = new byte[]{0x00, 0x00, 0x00, 0x01}; // initialising counter to 1
            counter = concatenate(iv, counter); // concatenates IV and 32-bit counter
        } else {
            GHASH(); // GHASH function for IV length
        }

        // encryption for the first counter to create the mask
        mask = new byte[counter.length];
        encryptor.encryptBlock(counter, mask);
        counter = addOne(counter); // incrementing counter
    }

    // This method adds a block of AAD for authentication
    @Override
    public void updateAAD(byte[] data) {
        tag = xor1(data, tag); // XOR AAD block with existing tag
        GF.multiplyByH(tag); // multiply tag by H
        lengthA += data.length * 8L; // gets the length of the AAD block using a long variable
    }

    // This method encrypts or decrypts a single block of data inline
    // storing the resulting plaintext or ciphertext bytes into the same data array
    @Override
    public void processBlock(byte[] data) {
        byte[] keyStream = new byte[16]; // initialising the key streams for each block
        encryptor.encryptBlock(counter, keyStream); // encryption for the first counter

        // decryption mode
        if (mode == CipherMode.DECRYPT) {
            tag = xor1(data, tag);
            GF.multiplyByH(tag);
        }

        xor2(data, keyStream); // XOR key streams together with corresponding plaintext block
        counter = addOne(counter); // incrementing counter

        // encryption mode
        if (mode == CipherMode.ENCRYPT) {
            tag = xor1(data, tag);
            GF.multiplyByH(tag);
        }
        lengthC += data.length * 8L; // gets the ciphertext length as a long variable
    }

    // This method calculates the final tag
    @Override
    public void finalise(byte[] out) {
        // Converting long variables to byte[] variables
        // concatenate length of the AAD and ciphertext together
        byte[] concat = concatenate(longToBytes(lengthA), longToBytes(lengthC));
        System.arraycopy(concat, 0, out, 0, 16); // copy byte[] concat to byte[] out
        xor2(out, tag); // XOR byte[] out with existing tag
        GF.multiplyByH(out); // multiply by H
        xor2(out, mask); // XOR initial mask with existing tag to produce final tag T
    }

    // This method finishes compares the final tag to the supplied tag array
    // and throws an InvalidTagException if the tag supplied as a parameter
    @Override
    public void verify(byte[] tag) throws InvalidTagException {
        byte[] output = new byte[16]; // new byte[] to check final tag
        finalise(output); // obtaining final tag by calling finalise
       if(!Arrays.equals(output, tag)) { // comparing tags
            throw new InvalidTagException();
       }
    }

    // This method is responsible for incrementing the counter by one
    public static byte[] addOne(byte[] data) {
        for (int i = data.length - 1; i >= 0; i--) {
            data[i]++;
            if (data[i] != 0) {
                return data; // No overflow, return incremented array
            }
        }
        // Overflow occurred, create a new byte array with one extra byte
        byte[] newData = new byte[data.length + 1];
        newData[0] = 1;
        return newData;
    }

    // This method was created to concatenate the IV and the counter together
    public byte[] concatenate(byte[] data1, byte[] data2) {
        byte[] concatenation = new byte[data1.length + data2.length];
        System.arraycopy(data1, 0, concatenation, 0, data1.length); // copy iv to concatenation
        System.arraycopy(data2, 0, concatenation, data1.length, data2.length); // copy counter to concatenation
        return concatenation;
    }

    // This function performs the xor operation in a loop
    // stores the result in second variable passed to the function
    public byte[] xor1(byte[] data1, byte[] data2) {
        for (int i = 0; i < data1.length; i++) {
            data2[i] ^= data1[i];
        }
        return data2;
    }

    // This function perform the xor operation in a loop
    // stores the result in first variable passed to the function
    public void xor2(byte[] data1, byte[] data2) {
        for (int i = 0; i < data1.length; i++) {
            data1[i] ^= data2[i];
        }
    }

    // This method converts a long variable to a byte[] variable using right shift
    private static byte[] longToBytes(long data) {
        return new byte[]{
                (byte) ((data >> 56) & 0xff),
                (byte) ((data >> 48) & 0xff),
                (byte) ((data >> 40) & 0xff),
                (byte) ((data >> 32) & 0xff),
                (byte) ((data >> 24) & 0xff),
                (byte) ((data >> 16) & 0xff),
                (byte) ((data >> 8) & 0xff),
                (byte) ((data >> 0) & 0xff),
        };
    }

    // GHASH function
    private void GHASH() {
        // for every block of 16 bytes add AAD block of data
        for (int i = 0; i < iv.length; i += 16) {
            xor1(Arrays.copyOfRange(iv, i, Math.min(i + 16, iv.length)), counter);
            GF.multiplyByH(counter);
        }
        // finalise tag
        byte[] out = concatenate(new byte[8], longToBytes(iv.length * 8L));
        xor2(counter, out);
        GF.multiplyByH(counter);
    }
}