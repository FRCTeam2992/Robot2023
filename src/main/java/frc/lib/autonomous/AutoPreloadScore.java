package frc.lib.autonomous;

public enum AutoPreloadScore {
    No_Preload("No Preload Score"),
    Hi_Cone("Hi Cone Preload");

    public String description;

    private AutoPreloadScore(String description) {
        this.description = description;
    }

}