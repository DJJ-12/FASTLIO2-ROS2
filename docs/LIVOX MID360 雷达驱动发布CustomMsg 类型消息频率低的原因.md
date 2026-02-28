LIVOX MID360 雷达驱动发布CustomMsg 类型消息频率慢的原因？

我发现雷达驱动发布livox_ros_driver2::msg::CustomMsg 类型的消息时，频率就降低到0.5hz ，这导致建图节点很久收不到雷达数据，kd树是空的，导致出现大量no effect points ,  然后，我跳出雷达驱动，人工发布CustomMsg  的消息，在终端执行如下命令：                                  
python3 << 'PYEOF'
import rclpy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
import time

rclpy.init()
node = rclpy.create_node('test_pub2')
pub = node.create_publisher(CustomMsg, '/test_custom2', 10)

msg = CustomMsg()
msg.point_num = 20000
msg.points = [CustomPoint() for _ in range(20000)]

print("Publishing CustomMsg with 20000 points at 10Hz...")
try:
    while True:
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        time.sleep(0.1)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()
PYEOF
发布的点云数量是20000  ；然后测试 频率是0.5  ：root@IRC302plnx:~# ros2 topic hz /test_custom2
average rate: 0.536
        min: 1.864s max: 1.870s std dev: 0.00335s window: 2
average rate: 0.529
        min: 1.864s max: 1.940s std dev: 0.03443s window: 3
average rate: 0.531
如果将 msg.point_num = 100 ，改成100 ，测试频率是10；
root@IRC302plnx:/mnt/ros2/fastlio2_ros2# ros2 topic hz /test_custom2 average rate: 9.674 min: 0.100s max: 0.109s std dev: 0.00213s window: 11 average rate: 9.683 min: 0.100s max: 0.109s std dev: 0.00164s window: 21 average rate: 9.695                                

总结：ROS2 节点之间传递消息，不是直接把内存里的数据复制过去，而是要经过一个"打包"的过程，把消息转换成可以在网络上传输的字节流，这个过程叫序列化。接收方收到字节流后再"解包"还原成消息，叫反序列化。
造成这个现象的原因是ROS2 底层负责序列化的组件 DDS（Data Distribution Service），默认用的是 FastRTPS，
为什么 CustomMsg 序列化慢？
这是因为CustomMsg 的结构是这样的：
CustomMsg {
    header
    lidar_id
    point_num: 20000
    points: [CustomPoint × 20000]  ← 变长数组！
}

points 是一个变长数组，里面有 20000 个点，每个点有 x、y、z、intensity、tag、offset_time 等字段。

序列化这 20000 个点，FastRTPS 需要：

计算总大小

分配内存缓冲区

逐个字段把每个点写入缓冲区

通过网络发送

在我的 ARM64 平台控制盒上，这个过程耗时超过了 100ms（一帧的时间），所以下一帧到来时上一帧还没处理完，消息积压，最终被丢弃，看起来就像频率降到了 0.5Hz。

为什么 PointCloud2 没有这个问题？

这是因为 PointCloud2 的结构是：

PointCloud2 {
    header
    height, width
    fields: [描述字段]
    data: uint8[] ← 原始字节数组
}

data 是一个原始字节数组，20000 个点的所有数据已经紧密排列在一起，序列化时直接把这块内存整体复制走就行了，不需要逐个字段处理，速度快很多。


更学术点来说就是：

第一步：雷达硬件内部

MID360 雷达每 100ms 扫描一圈，采集到约 20000 个激光点。每个点在雷达内部存储的原始数据是：
点1: x, y, z, 反射强度, 点质量标志(tag), 该点的采集时刻(绝对时间戳)
点2: x, y, z, 反射强度, tag, 绝对时间戳
...
点20000: ...
这 20000 个点在雷达内部内存里紧密排列成一块连续的原始字节数据。

第二步：雷达通过网线传给控制盒

雷达把这块原始字节数据打成 UDP 包，通过网线发给控制盒，这个过程非常快（百兆网线，20000个点约 500KB，不到 50ms 就传完了）。

第三步：控制盒上的 livox_ros_driver2 接收数据

驱动程序收到 UDP 包后，把原始字节数据解析成 ROS 消息，这里就出现了分叉：

如果 xfer_format=0，打包成 PointCloud2：PointCloud2.data = [把20000个点的所有字节直接塞进这个数组]

data 字段就是一块原始字节，20000个点 × 26字节/点 = 520000字节，全部连续存在一起，驱动只做了一次内存复制。

如果 xfer_format=1，打包成 CustomMsg：

CustomMsg.points = [
    CustomPoint{x, y, z, reflectivity, tag, line, offset_time},
    CustomPoint{x, y, z, reflectivity, tag, line, offset_time},
    ... × 20000
]

驱动需要把每个点的绝对时间戳换算成相对时间偏移（offset_time = 该点时间戳 - 帧头时间戳），然后逐个构造 CustomPoint 对象，填入 points 数组，这里做了 20000 次计算和赋值。

第四步：ROS2 通过 DDS 发布消息

这是关键区别所在。

消息构造完后，驱动调用 m_pub->publish(msg)，DDS 接管，把消息序列化成字节流，通过共享内存或网络传给订阅者。

PointCloud2 的序列化：

PointCloud2 内存布局:  [header][height][width][fields描述][data: 520000字节的原始数组]
DDS 序列化时，data 字段是一个 uint8[]，DDS 知道这是一块连续字节，直接整体 memcpy 走，一次操作搞定。

CustomMsg 的序列化：

CustomMsg 内存布局:  [header][lidar_id][point_num=20000][points: 20000个CustomPoint对象]
points 是一个 CustomPoint[]，这是 ROS2 的 IDL 变长数组，每个 CustomPoint 是一个独立的结构体对象。DDS（FastRTPS）序列化时，必须遍历这个数组，逐个把每个 CustomPoint 的每个字段写入缓冲区，因为 IDL 协议需要处理字节对齐、字节序等问题，不能简单地整体 memcpy。
在 x86 平台上这 20000 次遍历很快，但在这个 ARM64 平台上耗时超过了 100ms。



