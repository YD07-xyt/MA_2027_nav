#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <type_traits>

namespace utils {

// 非 ROS2 时钟句柄：16 字节平凡可拷贝非拥有值类型。
// 通过函数指针 + opaque ctx 注入时间源，不使用 std::function / 虚函数 / 互斥锁。
// 默认指向 steady_clock；now_ns_fn 为空时同样回退到 steady_clock。
inline auto steadyNowNs(const void* /*ctx*/) noexcept -> int64_t {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

class Clock {
public:
    using NowNsFn = int64_t (*)(const void*) noexcept;

    NowNsFn now_ns_fn = &steadyNowNs;
    const void* ctx = nullptr;

    static auto steady() noexcept -> Clock {
        return Clock {};
    }

    auto nowNs() const noexcept -> int64_t {
        return now_ns_fn != nullptr ? now_ns_fn(ctx) : steadyNowNs(nullptr);
    }

    auto nowSec() const noexcept -> double {
        return static_cast<double>(nowNs()) * 1e-9;
    }
};

static_assert(sizeof(Clock) == 2 * sizeof(void*), "utils::Clock must stay a 16-byte value type");
static_assert(std::is_trivially_copyable_v<Clock>, "utils::Clock must be trivially copyable");

// 原子时间源：ROS /clock 回调写入，任意线程只做一次 relaxed load 读取。
// asClock() 返回的 Clock 只持有 this 指针，调用方须保证 AtomicClockSource 生命周期覆盖 Clock 使用期。
class AtomicClockSource {
public:
    auto storeNs(int64_t ns) noexcept -> void {
        now_ns_.store(ns, std::memory_order_relaxed);
    }

    auto storeSec(double sec) noexcept -> void {
        storeNs(static_cast<int64_t>(sec * 1e9));
    }

    auto loadNs() const noexcept -> int64_t {
        return now_ns_.load(std::memory_order_relaxed);
    }

    auto asClock() const noexcept -> Clock {
        Clock clock;
        clock.now_ns_fn = &AtomicClockSource::nowNsThunk;
        clock.ctx = this;
        return clock;
    }

private:
    static auto nowNsThunk(const void* ctx) noexcept -> int64_t {
        return static_cast<const AtomicClockSource*>(ctx)->loadNs();
    }

    std::atomic<int64_t> now_ns_ {0};
};

// 手动时钟：离线/单测推进时间，asClock() 同样只做一次普通读取。
class ManualClock {
public:
    auto setNs(int64_t ns) noexcept -> void {
        now_ns_ = ns;
    }

    auto setSec(double sec) noexcept -> void {
        setNs(static_cast<int64_t>(sec * 1e9));
    }

    auto advanceNs(int64_t ns) noexcept -> void {
        now_ns_ += ns;
    }

    auto advanceSec(double sec) noexcept -> void {
        advanceNs(static_cast<int64_t>(sec * 1e9));
    }

    auto nowNs() const noexcept -> int64_t {
        return now_ns_;
    }

    auto nowSec() const noexcept -> double {
        return static_cast<double>(now_ns_) * 1e-9;
    }

    auto asClock() const noexcept -> Clock {
        Clock clock;
        clock.now_ns_fn = &ManualClock::nowNsThunk;
        clock.ctx = this;
        return clock;
    }

private:
    static auto nowNsThunk(const void* ctx) noexcept -> int64_t {
        return static_cast<const ManualClock*>(ctx)->now_ns_;
    }

    int64_t now_ns_ = 0;
};

} // namespace utils
