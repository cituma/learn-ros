#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <future>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>

#include "session.h"
#include "apollo_pointcloud.h"

class RecvMessage {
 public:
  RecvMessage(std::string ip = "127.0.0.1") : ip_(ip), recv_buf_size_(10 * 1024 * 1024) {
    recv_buf_.resize(recv_buf_size_);
  }

  ~RecvMessage() {
    Destroy();
    recv_buf_.clear();
  }

  int Init(uint16_t server_port) {
    std::cout << "SendMessage Init." << std::endl;

    int ret = InitSocket(server_port);
    if (ret) {
      std::cout << "InitSocket failed." << std::endl;
      return ret;
    }
    //async_read_ =
    //    std::async(std::launch::async, std::bind(&RecvMessage::RecvDataLoop, this));

    return 0;
  }

  void Destroy() {
    if (!connected_) return;
    connected_ = false;
    std::cout << "SendMessage Destroy." << std::endl;
    session_.Close();
    //async_read_.wait();
  }

  int InitSocket(uint16_t server_port) {
    struct sockaddr_in server_addr;
    server_addr.sin_addr.s_addr = inet_addr(ip_.c_str());
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port);

    session_.Socket(AF_INET, SOCK_STREAM, 0);
    if (session_.Connect((struct sockaddr*)&server_addr, sizeof(server_addr)) <
        0) {
      std::cout << "connect to server failed, " << strerror(errno) << std::endl;
      return -1;
    }
    connected_ = true;

    return 0;
  }

  void RecvDataLoop() {
    while (connected_) {
      int nbytes = RecvData();
      if (nbytes <= 0) {
          std::cout << "recv data failed!, recv size:" << nbytes << std::endl;
          return;
      }
      std::cout << "recv size:" << nbytes << std::endl;
    }
  }

  int RecvData() {
    size_t data_size = 0;
    int nbytes = SessionRecv(&data_size, sizeof(size_t));
    if (nbytes <= 0) {
        std::cout << "recv data size err:" << nbytes << std::endl;
        return nbytes;
    }
    std::cout << "data size:" << data_size << std::endl;

    if (data_size > recv_buf_size_) {
      recv_buf_size_ = data_size;
      recv_buf_.resize(data_size);
    }

    nbytes = SessionRecv(recv_buf_.data(), data_size);
    if (nbytes <= 0) {
        std::cout << "recv data err:" << nbytes << std::endl;
        return nbytes;
    }
    std::cout << "recv size:" << nbytes << std::endl;
    return nbytes;
  }

  int SessionRecv(void* buf, size_t size) {
    size_t recv_size = 0;
    while (recv_size != size) {
      char* data_buf = static_cast<char*>(buf) + recv_size;
      int nbytes =
          static_cast<int>(session_.Recv(data_buf, size - recv_size, 0));
      if (nbytes == 0) {
        std::cout << "server has been closed." << std::endl;
        session_.Close();
        return nbytes;
      }

      if (nbytes < 0) {
        std::cout << "receive message from server failed." << std::endl;
        session_.Close();
        return nbytes;
      }
      recv_size += nbytes;
    }

    return (int)recv_size;
  }

  const std::vector<char>& recv_buf() { return recv_buf_; }

 private:
  bool connected_ = false;
  Session session_;
  std::future<void> async_read_;

  size_t recv_buf_size_;
  std::vector<char> recv_buf_;

  std::string ip_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "apollo_client");
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(
      "pcl_output", 1);  // topic is "pcl_output"

  //
  uint16_t server_port = 11435;
  //std::string ip = "192.168.2.19";
  //std::string ip = "172.17.0.1";
  RecvMessage recv_msg;
  recv_msg.Init(server_port);

  ros::Rate loop_rate(1);
  while (ros::ok()) {
    int nbytes = recv_msg.RecvData();
    if(nbytes <= 0) {
      std::cout << "recv data err:" << nbytes << std::endl;
      break;
    }

    const std::vector<char>& buf = recv_msg.recv_buf();

    ApolloPointCloud apollo_pointcloud;
    apollo_pointcloud.Deserialize(buf.data(), nbytes);

    cloud.width = apollo_pointcloud.width;
    cloud.height = apollo_pointcloud.height;
    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size(); ++i) {
      cloud.points[i].x = apollo_pointcloud.points[i].x;
      cloud.points[i].y = apollo_pointcloud.points[i].y;
      cloud.points[i].z = apollo_pointcloud.points[i].z;
    }

    // Convert the cloud to ROS message
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "odom";

    pcl_pub.publish(output);
    ros::spinOnce();
    //loop_rate.sleep();
  }

  recv_msg.Destroy();
  return 0;
}

