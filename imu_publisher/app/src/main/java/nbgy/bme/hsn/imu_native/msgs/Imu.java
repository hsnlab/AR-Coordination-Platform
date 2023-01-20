package nbgy.bme.hsn.imu_native.msgs;


public class Imu {
    public Header header;
    public Quaternion orientation;
    public double[] orientation_covariance= {99999.9, 0.0, 0.0, 0.0, 99999.9, 0.0, 0.0, 0.0, 99999.9};
    public Vector3 angular_velocity;
    public double[] angular_velocity_covariance={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    public Vector3 linear_acceleration;
    public double[] linear_acceleration_covariance={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    public Imu(){}

    public Imu(Header header,Quaternion orientation,Vector3 angular_velocity, Vector3 linear_acceleration ){
        this.header=header;
        this.orientation= orientation;
        this.angular_velocity= angular_velocity;
        this.linear_acceleration= linear_acceleration;
    }

    public Imu(Header header,Vector3 angular_velocity, Vector3 linear_acceleration ){
        this.header=header;
        this.orientation= new Quaternion(0,0,0,1);
        this.angular_velocity= angular_velocity;
        this.linear_acceleration= linear_acceleration;
    }
}
