
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include "flyMS/ipc/redis/RedisSubscriber.h"
#include "spdlog/spdlog.h"

namespace flyMS {

/**
 * @brief A thread-safe queue for RedisSubscriberQueue to use as an intermediate receiving container
 *
 * @tparam MsgType The type of the message to be held in the queue
 */
template <typename MsgType>
struct RedisQueue {
 public:
  /**
   * @brief Construct a new Redis Queue object
   *
   * @param max_queue_size The maximum number of elements to store in the queue. The default is 0, which means no limit
   */
  RedisQueue(std::size_t max_queue_size = 0) : max_queue_size_(max_queue_size) {}

  /**
   * @brief Gets the front of the queue
   *
   * @return MsgType
   */
  MsgType front() {
    std::scoped_lock<std::mutex> lock(mutex_);
    return queue_.front();
  }

  /**
   * @brief Deletes the front-most point of the queue
   *
   */
  void pop() {
    std::scoped_lock<std::mutex> lock(mutex_);
    queue_.pop();
  }

  /**
   * @brief Moves the front most element out of the queue, and returns it. Also the element is deleted from the queue
   *
   * @return MsgType
   */
  MsgType pop_front() {
    std::scoped_lock<std::mutex> lock(mutex_);
    auto front = std::move(queue_.front());
    queue_.pop();
    return front;
  }

  /**
   * @brief Push an element to the end of the queue
   *
   * @param mav_msg
   */
  void push(MsgType& mav_msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (max_queue_size_ != 0 && queue_.size() >= max_queue_size_) {
      spdlog::warn("RedisQueue is full, dropping oldest message");
      queue_.pop();
    }

    queue_.push(mav_msg);
  }

  /**
   * @brief Get the size of the queue
   *
   * @return std::size_t
   */
  std::size_t size() const { return queue_.size(); }

  /**
   * @brief Check if the queue is empty
   *
   * @return true The queue is empty
   * @return false The queue contains elements
   */
  bool empty() const { return queue_.empty(); }

 private:
  std::size_t max_queue_size_;
  std::mutex mutex_;
  std::queue<MsgType> queue_;
};

class RedisSubscriberQueue {
 public:
  /**
   * @brief Construct a new RedisSubscriberQueue object
   *
   */
  RedisSubscriberQueue() = default;

  /**
   * @brief Destroy the RedisSubscriberQueue object
   *
   */
  ~RedisSubscriberQueue() = default;

  /**
   * @brief Registers a channel to listen to, and creates a queue that will receive messages from that queue
   *
   * @param channel The channel to listen to
   * @return std::shared_ptr<RedisQueue<std::string>>
   */
  std::shared_ptr<RedisQueue<std::string>> register_message(const std::string& channel) {
    if (redis_subscriber_) {
      spdlog::error("Tried to register message after RedisSubscriberQueue::start() was called");
      return nullptr;
    }

    auto redis_queue = std::make_shared<RedisQueue<std::string>>();
    redis_queues_.emplace(std::make_pair(channel, redis_queue));
    channels_.push_back(channel);
    return redis_queue;
  }

  /**
   * @brief Starts the RedisSubscriberQueue. All messages should be registered before this member function is called
   *
   */
  void start() {
    if (channels_.empty()) {
      spdlog::error(
          "Tried to start RedisSubscriberQueue with no messages registered! Call "
          "RedisSubscriberQueue::register_message() first");
    }

    auto callback_func = std::bind(&RedisSubscriberQueue::callback, this, std::placeholders::_1, std::placeholders::_2);
    redis_subscriber_ = std::make_unique<RedisSubscriber>(callback_func, channels_);
  }

 private:
  /**
   * @brief Simple callback function to provide to the RedisSubscriber. It will push the message to the appropriate
   * queue
   *
   * @param channel The channel the message was received on
   * @param msg The message content
   */
  void callback(std::string channel, std::string msg) {
    if (redis_queues_.find(channel) != redis_queues_.end()) {
      redis_queues_.find(channel)->second->push(msg);
    } else {
      spdlog::error("RedisSubscriberQueue: No queue registered for channel {}", channel);
    }
  }

  std::vector<std::string> channels_;                  //< The channels to listen to
  std::unique_ptr<RedisSubscriber> redis_subscriber_;  //< The subscriber which will receive the messages
  std::unordered_map<std::string, std::shared_ptr<RedisQueue<std::string>>>
      redis_queues_;  //< The queues to push the received messages to
};

}  // namespace flyMS
