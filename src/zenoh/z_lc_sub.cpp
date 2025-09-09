// z_sub_to_ros.cpp
#include "zenoh.hxx"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace zenoh;

class Z2Ros : public rclcpp::Node {
public:
  Z2Ros(const std::string& endpoint)
  : Node("z2ros_img") {
    // 로컬 ROS2 퍼블리셔
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("/carla/cam/front", 10);

    // Zenoh 세션(서버로 연결)
    auto j = std::string("{connect:{endpoints:[\"tcp/") + endpoint + "\"]}}";
    auto cfg = Config::from_json5(j);
    session_ = Session::open(std::move(cfg));

    sub_ = session_.declare_subscriber(KeyExpr("carla/cam/front"),
      [this](const Sample& s) {
        const auto& b = s.get_payload();

        // 1) 연속 뷰 시도 → 성공이면 포인터/길이 바로 사용 (추가 복사 0회)
        if (auto view = b.get_contiguous_view()) {
          const uint8_t* ptr = view->data;
          size_t len = view->len;

          auto msg = sensor_msgs::msg::Image();
          msg.width = 1280; msg.height = 720;
          msg.encoding = "rgb8";
          msg.step = 1280*3;
          msg.data.resize(len);                // ROS2 메시지 내부 저장소
          std::memcpy(msg.data.data(), ptr, len); // ← 여기서 1회 불가피한 복사
          // LoanedMessage가 가능하면 아래처럼 메시지 대여 후 직접 채우기 권장
          pub_->publish(std::move(msg));
        } else {
          // 2) 조각 스트리밍: 한 번에 하나씩 memcpy (큰 조각이면 L3 친화적)
          size_t total = b.len();
          auto msg = sensor_msgs::msg::Image();
          msg.width = 1280; msg.height = 720;
          msg.encoding = "rgb8";
          msg.step = 1280*3;
          msg.data.resize(total);
          auto it = b.slice_iter();
          size_t off = 0;
          while (auto sl = it.next()) {
            std::memcpy(msg.data.data()+off, sl->data, sl->len);
            off += sl->len;
          }
          pub_->publish(std::move(msg));
        }
      });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  Session session_;
  Subscriber sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Z2Ros>("SERVER_IP:7447");
  rclcpp::spin(node);
  rclcpp::shutdown();
}
