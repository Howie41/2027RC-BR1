#include "host_comm.hpp"
#include "host_msgs.hpp"

#include <algorithm>
#include <cassert>
#include <vector>

namespace {

using Pose = host_comm::VisionPose;
using Comm = host_comm::HostComm<Pose>;
constexpr Pose kPose{1.0F, -2.5F, 0.75F, 0x12345678U, 0x90ABCDEFU};
constexpr std::array<std::uint8_t, 28> kLegacyFrame{
    0xFF, 0xFE, 0x01, 0x14, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x20, 0xC0, 0x00, 0x00,
    0x40, 0x3F, 0x78, 0x56, 0x34, 0x12, 0xEF, 0xCD, 0xAB, 0x90, 0xB5, 0x85, 0xFD, 0xFB};
static_assert(Comm::build_frame(kPose) == kLegacyFrame);

struct Received {
    Pose pose{};
    std::size_t count{};
};

void receive(const Pose& pose, void* context) {
    auto& received = *static_cast<Received*>(context);
    received.pose = pose;
    ++received.count;
}

void test_crc() {
    using Region = frame::RegionTag<7>;
    using Modbus = frame::Crc<Region, frame::CrcType::Crc16Modbus>;
    using Existing = frame::Crc<Region>;
    constexpr std::array<std::uint8_t, 9> check{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    std::array<std::uint8_t, 2> crc{};
    Modbus::serialize(crc, check);
    assert((crc == std::array<std::uint8_t, 2>{0x37, 0x4B}));
    assert(Modbus::deserialize(crc, check));
    Existing::serialize(crc, check);
    assert((crc == std::array<std::uint8_t, 2>{0xB1, 0x29}));
    Modbus::serialize(crc, {});
    assert((crc == std::array<std::uint8_t, 2>{0xFF, 0xFF}));
}

void test_stream() {
    Received received{};
    Comm comm({receive}, &received);
    for (std::size_t split = 0; split <= kLegacyFrame.size(); ++split) {
        comm.reset();
        const auto before = received.count;
        comm.process_buffer(std::span{kLegacyFrame}.first(split));
        comm.process_buffer(std::span{kLegacyFrame}.subspan(split));
        assert(received.count == before + 1);
        assert(received.pose.to_tuple() == kPose.to_tuple());
    }
    comm.reset();
    for (const auto byte : kLegacyFrame)
        comm.process_buffer({&byte, 1});

    // 交叠帧头与大于解析缓存的连续输入。
    std::vector<std::uint8_t> stream{0xAA, 0xFF, 0xFF};
    for (std::size_t i = 0; i < 50; ++i)
        stream.insert(stream.end(), kLegacyFrame.begin(), kLegacyFrame.end());
    const auto before = received.count;
    assert(comm.process_buffer(stream) == 50);
    assert(received.count == before + 50);

    for (std::size_t index = 0; index < kLegacyFrame.size(); ++index) {
        auto corrupt = kLegacyFrame;
        corrupt[index] ^= 0x40;
        comm.reset();
        const auto count = received.count;
        comm.process_buffer(corrupt);
        comm.process_buffer(kLegacyFrame);
        assert(received.count == count + 1);
    }
    comm.reset();
    comm.process_buffer(std::span{kLegacyFrame}.first(10));
    comm.reset();
    const auto count = received.count;
    comm.process_buffer(std::span{kLegacyFrame}.subspan(10));
    assert(received.count == count);
    assert(comm.process_buffer(kLegacyFrame) == 1);
    Comm no_callback{};
    assert(no_callback.process_buffer(kLegacyFrame) == 1);
}

struct Empty {
    using Frame = host_comm::HostFrame<0x02>;
    constexpr Frame::DataTuple to_tuple() const { return {}; }
    static Empty from_tuple(const Frame::DataTuple&) { return {}; }
};

struct Large {
    using Frame = host_comm::HostFrame<0x03, frame::Field<std::array<std::uint8_t, 64>>>;
    std::array<std::uint8_t, 64> data{};
    Frame::DataTuple to_tuple() const { return {data}; }
    static Large from_tuple(const Frame::DataTuple& tuple) { return {std::get<0>(tuple)}; }
};

void test_multiple_messages() {
    using Multi = host_comm::HostComm<Pose, Empty, Large>;
    std::array<int, 3> counts{};
    Multi comm(
        {[](const Pose& pose, void* context) {
             assert(pose.to_tuple() == kPose.to_tuple());
             ++(*static_cast<std::array<int, 3>*>(context))[0];
         },
         [](const Empty&, void* context) { ++(*static_cast<std::array<int, 3>*>(context))[1]; },
         [](const Large& value, void* context) {
             assert(value.data[0] == 0xFF && value.data[63] == 0xFB);
             ++(*static_cast<std::array<int, 3>*>(context))[2];
         }},
        &counts);
    Large large{};
    large.data[0] = 0xFF;
    large.data[1] = 0xFE;
    large.data[62] = 0xFD;
    large.data[63] = 0xFB;
    const auto bytes = Multi::build_frame(large);
    static_assert(bytes.size() == 72);
    assert(bytes[3] == 64);
    assert(comm.process_buffer(bytes) == 1);
    assert(comm.process_buffer(Multi::build_frame(Empty{})) == 1);
    assert(comm.process_buffer(kLegacyFrame) == 1);
    assert((counts == std::array<int, 3>{1, 1, 1}));
}

} // namespace

int main() {
    test_crc();
    test_stream();
    test_multiple_messages();
}
