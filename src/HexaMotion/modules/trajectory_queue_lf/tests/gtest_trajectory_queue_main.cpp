#include "gtest/gtest.h"
#include "TrajectoryQueue.h" // The refactored header
#include <thread>
#include <vector>
#include <numeric>

// Use the fully refactored queue from the RDT namespace
using TestQueue = RDT::TrajectoryQueue<int, 4>;

// Test fixture for a clean queue in each test
class TrajectoryQueueTest : public ::testing::Test {
protected:
    TestQueue queue;
};

// REQ-TQLF-02, REQ-TQLF-04
TEST_F(TrajectoryQueueTest, PushAndPopSingleElement) {
    ASSERT_TRUE(queue.try_push(42));
    ASSERT_EQ(queue.size(), 1);

    // MODIFIED: Check Result<T, E> instead of bool/out-param
    auto pop_result = queue.try_pop();
    ASSERT_TRUE(pop_result.isSuccess());
    EXPECT_EQ(pop_result.value(), 42);

    ASSERT_EQ(queue.size(), 0);
    ASSERT_TRUE(queue.empty());
}

// REQ-TQLF-04
TEST_F(TrajectoryQueueTest, PopFromEmptyQueue) {
    // MODIFIED: Check for error state and correct ErrorCode
    auto pop_result = queue.try_pop();
    ASSERT_TRUE(pop_result.isError());
    EXPECT_EQ(pop_result.error(), RDT::ErrorCode::IndexOutOfRange);
    ASSERT_TRUE(queue.empty());
}

// REQ-TQLF-04
TEST_F(TrajectoryQueueTest, PeekFromEmptyQueue) {
    // MODIFIED: Check for error state and correct ErrorCode
    auto peek_result = queue.try_peek();
    ASSERT_TRUE(peek_result.isError());
    EXPECT_EQ(peek_result.error(), RDT::ErrorCode::IndexOutOfRange);
}

// REQ-TQLF-03
TEST_F(TrajectoryQueueTest, PushToFullQueue) {
    ASSERT_TRUE(queue.try_push(1));
    ASSERT_TRUE(queue.try_push(2));
    ASSERT_TRUE(queue.try_push(3));
    ASSERT_EQ(queue.size(), 3);

    // The queue's capacity is 4, but one slot is always left empty to distinguish
    // a full queue from an empty one. So, we can only push 3 items.
    ASSERT_FALSE(queue.try_push(4));
    ASSERT_EQ(queue.size(), 3);
}

// REQ-TQLF-02
TEST_F(TrajectoryQueueTest, PeekDoesNotRemoveElement) {
    queue.try_push(100);
    ASSERT_EQ(queue.size(), 1);

    // MODIFIED: Check Result<T, E> from try_peek
    auto peek_result1 = queue.try_peek();
    ASSERT_TRUE(peek_result1.isSuccess());
    EXPECT_EQ(peek_result1.value(), 100);

    // Verify element is still there
    ASSERT_EQ(queue.size(), 1);
    auto peek_result2 = queue.try_peek();
    ASSERT_TRUE(peek_result2.isSuccess());
    EXPECT_EQ(peek_result2.value(), 100);

    // Pop the element and verify it's the same one
    auto pop_result = queue.try_pop();
    ASSERT_TRUE(pop_result.isSuccess());
    EXPECT_EQ(pop_result.value(), 100);
    ASSERT_TRUE(queue.empty());
}

// REQ-TQLF-02
TEST_F(TrajectoryQueueTest, FifoOrderIsPreserved) {
    queue.try_push(1);
    queue.try_push(2);
    queue.try_push(3);

    // MODIFIED: Check Result<T, E> for each pop
    auto res1 = queue.try_pop();
    ASSERT_TRUE(res1.isSuccess());
    EXPECT_EQ(res1.value(), 1);

    auto res2 = queue.try_pop();
    ASSERT_TRUE(res2.isSuccess());
    EXPECT_EQ(res2.value(), 2);

    auto res3 = queue.try_pop();
    ASSERT_TRUE(res3.isSuccess());
    EXPECT_EQ(res3.value(), 3);

    ASSERT_TRUE(queue.empty());
}

// REQ-TQLF-02
TEST_F(TrajectoryQueueTest, WrapAround) {
    // Fill the queue
    queue.try_push(10);
    queue.try_push(20);
    queue.try_push(30);

    // Empty it
    queue.try_pop();
    queue.try_pop();
    queue.try_pop();

    // Fill it again, causing tail to wrap around
    queue.try_push(40);
    queue.try_push(50);
    queue.try_push(60);

    // Check order
    auto res1 = queue.try_pop();
    ASSERT_TRUE(res1.isSuccess());
    EXPECT_EQ(res1.value(), 40);

    auto res2 = queue.try_pop();
    ASSERT_TRUE(res2.isSuccess());
    EXPECT_EQ(res2.value(), 50);

    auto res3 = queue.try_pop();
    ASSERT_TRUE(res3.isSuccess());
    EXPECT_EQ(res3.value(), 60);
}

// REQ-TQLF-05
TEST_F(TrajectoryQueueTest, Clear) {
    queue.try_push(1);
    queue.try_push(2);
    ASSERT_EQ(queue.size(), 2);

    queue.clear();
    ASSERT_EQ(queue.size(), 0);
    ASSERT_TRUE(queue.empty());

    // Check that popping fails after clearing
    auto pop_result = queue.try_pop();
    ASSERT_TRUE(pop_result.isError());

    // Check that we can push again
    ASSERT_TRUE(queue.try_push(100));
    ASSERT_EQ(queue.size(), 1);
}

// REQ-TQLF-01, REQ-TQLF-NFR-01
TEST(TrajectoryQueueSpscTest, SpscThreadSafety) {
    RDT::TrajectoryQueue<int, 1024> spsc_queue;
    const int num_items = 100000;
    std::atomic<bool> producer_ready = false;

    std::thread producer([&]() {
        producer_ready = true;
        for (int i = 0; i < num_items; ++i) {
            while (!spsc_queue.try_push(i)) {
                // Spin wait for space
            }
        }
    });

    std::thread consumer([&]() {
        while (!producer_ready) { /* wait for producer to start */ }
        std::vector<int> consumed_items;
        consumed_items.reserve(num_items);
        for (int i = 0; i < num_items; ++i) {
            // MODIFIED: Use new try_pop() signature
            RDT::Result<int, RDT::ErrorCode> result = RDT::Result<int, RDT::ErrorCode>::Failure(RDT::ErrorCode::IndexOutOfRange);
            while (result.isError()) {
                result = spsc_queue.try_pop();
            }
            consumed_items.push_back(result.value());
        }

        // Verify that all items were consumed in the correct order
        std::vector<int> expected_items(num_items);
        std::iota(expected_items.begin(), expected_items.end(), 0);
        ASSERT_EQ(consumed_items.size(), num_items);
        ASSERT_EQ(consumed_items, expected_items);
    });

    producer.join();
    consumer.join();
}