// publisher.cpp
#include "zenoh.hxx"
#include <cstdint>
#include <cstring>
using namespace zenoh;

int main() {
  auto cfg = Config::create_default();
  // 필요 시 listen 또는 connect 명시: cfg.parse("...json5...");
  auto session = Session::open(std::move(cfg));
  auto pub = session.declare_publisher(KeyExpr("carla/cam/front"));

  // 예: 카메라 1280x720 RGB
  const size_t W = 1280, H = 720, C = 3;
  const size_t N = W*H*C;

  while (true) {
    // (핫패스) 카메라 콜백에서 바로 buf를 채웠다고 가정
    uint8_t* buf = new uint8_t[N];       // 애플리케이션 버퍼
    // ... fill buf with image bytes ...

    // ★ 복사 없이 소유권을 Zenoh에 이전(전송 끝나면 deleter로 해제)
    Bytes payload(buf, N, [](uint8_t* p){ delete[] p; });

    PublisherPutOptions popts;
    popts.set_priority(Priority::Z_PRIORITY_INTERACTIVE_HIGH);
    popts.set_congestion_control(CongestionControl::Z_CONGESTION_CONTROL_DROP); // 지연 우선

    pub.put(std::move(payload), &popts); // 앱→Zenoh 추가 복사 0회
  }
}
