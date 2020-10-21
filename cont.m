m1 = 0;
m2 = 0;
rosshutdown
rosinit('mypc');
sub1 = rossubscriber('/mpuReadings');
sub2 = rossubscriber('/Ultrasonic');
pub = rospublisher('/motors','std_msgs/UInt8MultiArray');
while (1)
    mpu = receive(sub1,5);
    mpu.Data
    r = round(mpu.Data(1)*10);
    p = round(mpu.Data(2));
    y = mpu.Data(3);
    dr = mpu.Data(4);
    dp = mpu.Data(5);
    dy = mpu.Data(6);

    ult = receive(sub2,5);
    u=ult.Data
    m1=cast(r,'uint8');
    m2=cast(u,'uint8');
    if m1 > 255 | m1 < 0
        m1 = 255;
    end
    if m2 > 255 
        m1 = 255;
    end
    pause(0.02);
    msg = rosmessage(pub);
    msg.Data = [10,20,3,4,5,6,7,8]
    send(pub,msg);
end
