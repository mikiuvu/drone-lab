#define main companion_runner_entry
#include "../companion/src/runner.cpp"
#undef main
#include <cassert>

int main() {
  Udp receiver(0),sender(0);
  sockaddr_in address{};socklen_t length=sizeof(address);
  assert(getsockname(receiver.fd,reinterpret_cast<sockaddr*>(&address),&length)==0);
  companion::Estimate estimate;
  estimate.sim_time_us=1000000;
  estimate.pose={2,3,1.2,0,0,0};
  estimate.covariance.setIdentity();estimate.covariance*=.001;
  estimate.z_valid=true;estimate.xy_valid=false;
  odometry(sender,ntohs(address.sin_port),estimate);
  std::string bytes;
  for(int i=0;i<100&&bytes.empty();++i){bytes=receiver.receive();std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  assert(!bytes.empty());
  mavlink_message_t message{};mavlink_status_t status{};bool decoded=false;
  for(unsigned char byte:bytes)if(mavlink_parse_char(MAVLINK_COMM_0,byte,&message,&status))decoded=true;
  assert(decoded&&message.msgid==MAVLINK_MSG_ID_ODOMETRY);
  mavlink_odometry_t value{};mavlink_msg_odometry_decode(&message,&value);
  assert(std::abs(value.z+1.2)<1e-6);
  assert(value.pose_covariance[0]>=10000&&value.pose_covariance[6]>=10000);
  assert(std::abs(value.pose_covariance[11]-.001)<1e-8);
  assert(value.pose_covariance[2]==0&&value.pose_covariance[7]==0);
  assert(value.quality>0);
  assert(value.velocity_covariance[0]>=10000&&value.velocity_covariance[6]>=10000);
  assert(std::abs(value.velocity_covariance[11]-.001)<1e-8);
}
