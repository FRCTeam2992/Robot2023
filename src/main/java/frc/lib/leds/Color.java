package frc.lib.leds;

public class Color {

    private int mR = 0;
    private int mG = 0;
    private int mB = 0;

    public Color(int r, int g, int b) {
        mR = r;
        mG = g;
        mB = b;
    }

    public int r() {
        return mR;
    }

    public int g() {
        return mG;
    }

    public int b() {
        return mB;
    }
}
