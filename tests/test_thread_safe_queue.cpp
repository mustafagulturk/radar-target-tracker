#include "radar/thread_safe_queue.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

namespace radar {
namespace {

using namespace std::chrono_literals;

TEST(ThreadSafeQueue, FifoOrder) {
    ThreadSafeQueue<int> q(10);
    for (int i = 0; i < 5; ++i) {
        EXPECT_TRUE(q.push(i));
    }
    EXPECT_EQ(q.size(), 5U);
    for (int i = 0; i < 5; ++i) {
        const auto v = q.tryPop();
        ASSERT_TRUE(v.has_value());
        EXPECT_EQ(*v, i);
    }
    EXPECT_FALSE(q.tryPop().has_value());
    EXPECT_EQ(q.dropped(), 0U);
}

TEST(ThreadSafeQueue, ZeroCapacityBecomesOne) {
    ThreadSafeQueue<int> q(0);
    EXPECT_EQ(q.capacity(), 1U);
}

TEST(ThreadSafeQueue, DropsOldestWhenFull) {
    ThreadSafeQueue<int> q(3);
    for (int i = 0; i < 7; ++i) {
        EXPECT_TRUE(q.push(i));  // never blocks, never refuses while open
    }
    EXPECT_EQ(q.size(), 3U);
    EXPECT_EQ(q.dropped(), 4U);
    EXPECT_EQ(*q.tryPop(), 4);
    EXPECT_EQ(*q.tryPop(), 5);
    EXPECT_EQ(*q.tryPop(), 6);
}

TEST(ThreadSafeQueue, SupportsMoveOnlyTypes) {
    ThreadSafeQueue<std::unique_ptr<int>> q(2);
    q.push(std::make_unique<int>(42));
    const auto v = q.pop();
    ASSERT_TRUE(v.has_value());
    EXPECT_EQ(**v, 42);
}

TEST(ThreadSafeQueue, CloseRejectsPushButDrainsRemaining) {
    ThreadSafeQueue<int> q(4);
    q.push(1);
    q.push(2);
    q.close();
    EXPECT_TRUE(q.closed());
    EXPECT_FALSE(q.push(3));
    EXPECT_EQ(*q.pop(), 1);
    EXPECT_EQ(*q.pop(), 2);
    EXPECT_FALSE(q.pop().has_value());  // closed and empty: does not block
}

TEST(ThreadSafeQueue, CloseWakesBlockedConsumers) {
    ThreadSafeQueue<int> q(4);
    std::atomic<int> woke{0};
    std::vector<std::thread> consumers;
    for (int i = 0; i < 3; ++i) {
        consumers.emplace_back([&] {
            if (!q.pop().has_value()) {
                ++woke;
            }
        });
    }
    std::this_thread::sleep_for(50ms);
    EXPECT_EQ(woke.load(), 0);
    const auto t0 = std::chrono::steady_clock::now();
    q.close();
    for (auto& t : consumers) {
        t.join();
    }
    EXPECT_EQ(woke.load(), 3);
    EXPECT_LT(std::chrono::steady_clock::now() - t0, 2s);
}

TEST(ThreadSafeQueue, PopWaitsForProducer) {
    ThreadSafeQueue<int> q(4);
    std::thread producer([&] {
        std::this_thread::sleep_for(20ms);
        q.push(99);
    });
    const auto v = q.pop();
    producer.join();
    ASSERT_TRUE(v.has_value());
    EXPECT_EQ(*v, 99);
}

TEST(ThreadSafeQueue, PopForTimesOut) {
    ThreadSafeQueue<int> q(4);
    const auto t0 = std::chrono::steady_clock::now();
    EXPECT_FALSE(q.popFor(30ms).has_value());
    const auto elapsed = std::chrono::steady_clock::now() - t0;
    EXPECT_GE(elapsed, 25ms);
    EXPECT_LT(elapsed, 2s);

    q.push(5);
    const auto v = q.popFor(1s);
    ASSERT_TRUE(v.has_value());
    EXPECT_EQ(*v, 5);
}

TEST(ThreadSafeQueue, MultiProducerMultiConsumerDeliversEachItemOnce) {
    constexpr int kProducers = 4;
    constexpr int kConsumers = 4;
    constexpr int kPerProducer = 5000;
    constexpr int kTotal = kProducers * kPerProducer;

    // Capacity large enough that nothing is ever dropped.
    ThreadSafeQueue<int> q(kTotal);
    std::vector<std::atomic<int>> seen(kTotal);
    for (auto& s : seen) {
        s.store(0);
    }

    std::vector<std::thread> consumers;
    std::atomic<int> received{0};
    for (int c = 0; c < kConsumers; ++c) {
        consumers.emplace_back([&] {
            while (auto v = q.pop()) {
                seen[static_cast<std::size_t>(*v)].fetch_add(1);
                ++received;
            }
        });
    }

    std::vector<std::thread> producers;
    for (int p = 0; p < kProducers; ++p) {
        producers.emplace_back([&q, p] {
            for (int i = 0; i < kPerProducer; ++i) {
                q.push(p * kPerProducer + i);
            }
        });
    }
    for (auto& t : producers) {
        t.join();
    }
    q.close();
    for (auto& t : consumers) {
        t.join();
    }

    EXPECT_EQ(q.dropped(), 0U);
    EXPECT_EQ(received.load(), kTotal);
    for (int i = 0; i < kTotal; ++i) {
        ASSERT_EQ(seen[static_cast<std::size_t>(i)].load(), 1) << "item " << i;
    }
}

TEST(ThreadSafeQueue, ConcurrentOverflowAccountsForEveryItem) {
    // With a tiny queue, every pushed item is either consumed or counted as dropped.
    constexpr int kProducers = 3;
    constexpr int kPerProducer = 3000;
    ThreadSafeQueue<int> q(8);
    std::atomic<int> received{0};
    std::thread consumer([&] {
        while (q.pop()) {
            ++received;
        }
    });
    std::vector<std::thread> producers;
    for (int p = 0; p < kProducers; ++p) {
        producers.emplace_back([&] {
            for (int i = 0; i < kPerProducer; ++i) {
                q.push(i);
            }
        });
    }
    for (auto& t : producers) {
        t.join();
    }
    q.close();
    consumer.join();
    EXPECT_EQ(static_cast<std::size_t>(received.load()) + q.dropped(),
              static_cast<std::size_t>(kProducers * kPerProducer));
}

}  // namespace
}  // namespace radar
