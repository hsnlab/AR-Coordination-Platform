package nbgy.bme.hsn.imu_native.msgs;

public class Time {
    public int secs;
    public int nsecs;

    public Time() {
    }

    public Time(int secs, int nsecs) {
        this.secs = secs;
        this.nsecs = nsecs;
    }

    public Time(long nanos){
        secs= (int) (nanos/ 1000000000);
        nsecs= (int) (nanos - secs* 1000000000);
    }
}