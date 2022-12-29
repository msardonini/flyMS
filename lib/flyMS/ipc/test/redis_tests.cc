#include <condition_variable>
#include <mutex>
#include <thread>

#include "flyMS/ipc/redis/RedisPublisher.h"
#include "flyMS/ipc/redis/RedisSubscriber.h"
#include "flyMS/ipc/redis/RedisSubscriberQueue.h"
#include "gtest/gtest.h"

static constexpr auto kREDIS_TIMEOUT = std::chrono::seconds(10);
class RedisTestFixture : public ::testing::Test {
 public:
  RedisTestFixture() = default;

  // std::unique_ptr<flyMS::RedisSubscriber> sub;
  // std::unique_ptr<flyMS::RedisPublisher> pub;

  std::function<void(const sw::redis::Error&)> timeout_callback = [](const auto err) {};

  // For thread synchronization
  std::mutex cv_mutex;
  std::condition_variable cond_var;

  static constexpr int num_msgs = 10;

  static constexpr char test_channel[] = "test_redis2";
  static constexpr char test_message[] = "Hello Redis!";
};

TEST_F(RedisTestFixture, SendAndReceive) {
  std::string output_str;
  auto callback = [this, &output_str](std::string chan, std::string msg) {
    output_str = msg;
    this->cond_var.notify_one();
  };

  auto sub =
      std::make_unique<flyMS::RedisSubscriber>(callback, std::vector<std::string>{test_channel}, timeout_callback);
  auto pub = std::make_shared<flyMS::RedisPublisher>();

  auto pub_msg = [](auto pub, auto chan, auto msg) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    pub->publish(chan, msg);
  };

  pub->publish(test_channel, test_message);
  std::thread pub_thread(pub_msg, pub, test_channel, test_message);

  std::unique_lock<std::mutex> lock(cv_mutex);
  if (cond_var.wait_for(lock, kREDIS_TIMEOUT) == std::cv_status::timeout) {
    FAIL() << "Timed out waiting for Redis Message!";
  }

  EXPECT_EQ(output_str, test_message);
  pub_thread.join();
}

TEST_F(RedisTestFixture, SendAndReceiveMany) {
  const std::string input_str(test_message);
  const std::string test_channel2 = "test_channel2";

  std::vector<std::string> output_strings_chan0;
  std::vector<std::string> output_strings_chan1;

  auto callback = [&, this](std::string chan, std::string msg) {
    if (chan == test_channel) {
      output_strings_chan0.push_back(msg);
    } else if (chan == test_channel2) {
      output_strings_chan1.push_back(msg);
    } else {
      FAIL() << "Received message on unknown channel: " << chan;
    }
  };

  auto sub = std::make_unique<flyMS::RedisSubscriber>(callback, std::vector<std::string>{test_channel, test_channel2},
                                                      timeout_callback);
  auto pub = std::make_unique<flyMS::RedisPublisher>();

  // Simple lambda to geneate a unique string given an input number
  auto generate_msg = [input_str](int msg_number) { return input_str + std::string(" ") + std::to_string(msg_number); };

  for (auto msg_num = 0; msg_num < num_msgs; msg_num++) {
    auto& channel = (msg_num % 2 == 0) ? test_channel : test_channel2;
    pub->publish(channel, generate_msg(msg_num));
  }

  // Wait for all our messages to arrive
  auto start = std::chrono::steady_clock::now();
  while (output_strings_chan0.size() + output_strings_chan1.size() < num_msgs) {
    if (std::chrono::steady_clock::now() > start + kREDIS_TIMEOUT) {
      FAIL() << " Timeout waiting for all messages to arrive!";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  EXPECT_EQ(output_strings_chan0.size(), num_msgs / 2);
  for (auto msg_num = 0; msg_num < num_msgs / 2; msg_num++) {
    EXPECT_EQ(output_strings_chan0[msg_num], generate_msg(msg_num * 2));
    EXPECT_EQ(output_strings_chan1.at(msg_num), generate_msg(msg_num * 2 + 1));
  }
}

TEST_F(RedisTestFixture, SendAndReceiveQueue) {
  flyMS::RedisSubscriberQueue redis_sub_queue;
  auto queue = redis_sub_queue.register_message(test_channel);
  redis_sub_queue.start();

  auto pub = std::make_unique<flyMS::RedisPublisher>();
  pub->publish(test_channel, test_message);

  auto start = std::chrono::steady_clock::now();
  while (queue->empty()) {
    if (std::chrono::steady_clock::now() > start + kREDIS_TIMEOUT) {
      FAIL() << " Timeout waiting for message to arrive!";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  EXPECT_EQ(queue->front(), test_message);
}
