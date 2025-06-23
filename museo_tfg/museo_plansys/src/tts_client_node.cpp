#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "my_interfaces/srv/text_to_speech.hpp"

class TTSClient : public rclcpp::Node
{
public:
  TTSClient() : Node("tts_client_node")
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/say_text", 10,
      std::bind(&TTSClient::callback, this, std::placeholders::_1));

    client_ = this->create_client<my_interfaces::srv::TextToSpeech>("tts_service");
  }

private:
  void callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "❌ Servicio TTS no disponible");
      return;
    }

    auto request = std::make_shared<my_interfaces::srv::TextToSpeech::Request>();
    request->text = msg->data;

    auto future = client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
      auto result = future.get();
      if (!result->success) {
        RCLCPP_WARN(get_logger(), "⚠️ Falló el TTS: %s", result->debug.c_str());
      }
    } else {
      RCLCPP_ERROR(get_logger(), "⏳ Timeout esperando TTS");
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Client<my_interfaces::srv::TextToSpeech>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TTSClient>());
  rclcpp::shutdown();
  return 0;
}
