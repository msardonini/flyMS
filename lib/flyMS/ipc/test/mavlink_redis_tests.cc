#include "flyMS/ipc/mavlink/MavlinkRedisPub.h"
#include "flyMS/ipc/mavlink/MavlinkRedisSub.h"
#include "flyMS/ipc/mavlink/MavlinkRedisSubQueue.h"
#include "gtest/gtest.h"

class MavlinkRedisTestFixture : public ::testing::Test {
 public:
  MavlinkRedisTestFixture() = default;

  static constexpr int num_msgs = 10;
  static constexpr char test_channel[] = "test_redis_mavlink";
};

TEST_F(MavlinkRedisTestFixture, Ctor) {
  flyMS::MavlinkRedisSub sub(test_channel);
  flyMS::MavlinkRedisPub pub;
}

TEST_F(MavlinkRedisTestFixture, SendAndReceive) {
  flyMS::MavlinkRedisSub sub(test_channel);
  flyMS::MavlinkRedisPub pub;

  mavlink_odometry_t odom_msg_send, odom_msg_receive;
  std::mutex cv_mutex;
  std::condition_variable cond_var;

  auto odom_callback = [&](auto& odom) {
    odom_msg_receive = odom;
    cond_var.notify_one();
  };

  // Fill with some random values
  odom_msg_send.x = 10;
  odom_msg_send.y = 3.14;
  odom_msg_send.q[3] = 0.775;

  sub.register_message<mavlink_odometry_t, MAVLINK_MSG_ID_ODOMETRY>(odom_callback, &mavlink_msg_odometry_decode);

  pub.publish<mavlink_odometry_t>(test_channel, odom_msg_send, &mavlink_msg_odometry_encode);

  std::unique_lock<std::mutex> lock(cv_mutex);

  if (cond_var.wait_for(lock, std::chrono::seconds(3)) == std::cv_status::timeout) {
    FAIL() << "Timed out waiting for Redis Message!";
  }

  // Make sure our values match
  EXPECT_EQ(odom_msg_send.x, odom_msg_receive.x);
  EXPECT_EQ(odom_msg_send.y, odom_msg_receive.y);
  EXPECT_EQ(odom_msg_send.q[3], odom_msg_receive.q[3]);
}

TEST_F(MavlinkRedisTestFixture, SendAndReceiveWithQueue) {
  flyMS::MavlinkRedisPub pub;

  mavlink_odometry_t odom_msg_send;
  flyMS::MavlinkRedisSubQueue sub_queue(test_channel);

  // Fill with some values
  odom_msg_send.x = 10;
  odom_msg_send.y = 3.14;
  odom_msg_send.q[3] = 0.775;

  auto odom_queue =
      sub_queue.register_message_with_queue<mavlink_odometry_t, MAVLINK_MSG_ID_ODOMETRY>(&mavlink_msg_odometry_decode);
  pub.publish<mavlink_odometry_t>(test_channel, odom_msg_send, &mavlink_msg_odometry_encode);

  // Wait for message to show up
  auto start = std::chrono::system_clock::now();
  while (odom_queue->size() == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (std::chrono::system_clock::now() > start + std::chrono::seconds(3)) {
      FAIL() << "Timed out waiting for Redis Message!";
    }
  }

  auto odom_msg_receive = odom_queue->pop_front();

  // Make sure our values match
  EXPECT_EQ(odom_msg_send.x, odom_msg_receive.x);
  EXPECT_EQ(odom_msg_send.y, odom_msg_receive.y);
  EXPECT_EQ(odom_msg_send.q[3], odom_msg_receive.q[3]);
}

TEST_F(MavlinkRedisTestFixture, SendAndReceiveManyWithQueue) {
  flyMS::MavlinkRedisPub pub;

  mavlink_odometry_t odom_msg_send;
  flyMS::MavlinkRedisSubQueue sub_queue(test_channel);

  // Fill with some values
  odom_msg_send.x = 10;
  odom_msg_send.y = 3.14;
  odom_msg_send.q[3] = 0.775;

  auto odom_queue =
      sub_queue.register_message_with_queue<mavlink_odometry_t, MAVLINK_MSG_ID_ODOMETRY>(&mavlink_msg_odometry_decode);

  for (auto i = 0; i < num_msgs; i++) {
    pub.publish<mavlink_odometry_t>(test_channel, odom_msg_send, &mavlink_msg_odometry_encode);
  }

  // Wait for message to show up
  auto start = std::chrono::system_clock::now();
  while (odom_queue->size() < num_msgs) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (std::chrono::system_clock::now() > start + std::chrono::seconds(3)) {
      FAIL() << "Timed out waiting for Redis Message!";
    }
  }

  EXPECT_EQ(odom_queue->size(), num_msgs);

  for (auto i = 0; i < num_msgs; i++) {
    auto odom_msg_receive = odom_queue->pop_front();

    // Make sure our values match
    EXPECT_EQ(odom_msg_send.x, odom_msg_receive.x);
    EXPECT_EQ(odom_msg_send.y, odom_msg_receive.y);
    EXPECT_EQ(odom_msg_send.q[3], odom_msg_receive.q[3]);
  }
}
