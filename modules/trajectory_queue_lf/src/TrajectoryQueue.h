// TrajectoryQueue.h
#ifndef TRAJECTORYQUEUE_H
#define TRAJECTORYQUEUE_H

#pragma once

#include "ErrorCode.h"
#include "result.h"

#include <atomic>
#include <cstddef>     // For std::size_t
#include <memory>      // For std::unique_ptr
#include <type_traits> // For std::is_nothrow_..._assignable/constructible

namespace RDT {

/**
 * @brief A lock-free, Single-Producer, Single-Consumer (SPSC) ring buffer queue.
 * @tparam T The type of elements stored in the queue.
 * @tparam Capacity The maximum number of elements the queue can hold.
 *
 * @section contract Contract
 * This queue guarantees thread-safe, wait-free operations **only** under the strict
 * SPSC contract:
 * - A single, designated thread may call the `try_push()` methods (the "Producer").
 * - A single, designated thread may call the `try_pop()` and `try_peek()` methods (the "Consumer").
 * - `empty()`, `size()`, and `clear()` have specific thread-safety guarantees outlined in their
 *   respective documentation.
 *
 * @section assumptions Assumptions
 * - `T` must be default-constructible, copy-assignable, and move-assignable.
 *
 * @section limitations Limitations
 * - The `Capacity` template parameter **MUST** be a power of 2. This is enforced by a `static_assert`.
 * - This queue is **NOT** safe for Multiple-Producer, Multiple-Consumer (MPMC) or
 *   Multiple-Producer, Single-Consumer (MPSC) scenarios.
 *
 * @section safety Safety
 * Incorrect use (violating the SPSC contract) will lead to data races and undefined behavior.
 * The `clear()` method is not safe to be called concurrently by both producer and consumer.
 */
template <typename T, std::size_t Capacity = 256>
class TrajectoryQueue {
    // Ensure Capacity is a power of 2 for efficient modulo operations using bitwise AND
    static_assert((Capacity > 0) && ((Capacity & (Capacity - 1)) == 0),
                  "TrajectoryQueue: Capacity must be a power of 2 and greater than 0.");

public:
    /**
     * @brief Constructs an empty queue and allocates the underlying buffer.
     */
    TrajectoryQueue() : head_(0), tail_(0) {
        buffer_ = std::make_unique<T[]>(Capacity);
    }

    // This class manages a unique resource (the buffer) and thread-sensitive atomic indices,
    // making it non-copyable and non-movable.
    TrajectoryQueue(const TrajectoryQueue&) = delete;
    TrajectoryQueue& operator=(const TrajectoryQueue&) = delete;
    TrajectoryQueue(TrajectoryQueue&&) = delete;
    TrajectoryQueue& operator=(TrajectoryQueue&&) = delete;

    /**
     * @brief Attempts to push an item to the back of the queue (copy).
     * @param item The item to push.
     * @return `true` if the item was successfully pushed, `false` if the queue was full.
     * @note Contract: **Producer thread only.**
     */
    [[nodiscard]] bool try_push(const T& item) noexcept(std::is_nothrow_copy_assignable_v<T>) {
        const auto current_tail = tail_.load(std::memory_order_relaxed);
        const auto next_tail = increment(current_tail);

        if (next_tail == head_.load(std::memory_order_acquire)) {
            return false; // Queue is full
        }

        buffer_[current_tail] = item;
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }

    /**
     * @brief Attempts to push an item to the back of the queue (move).
     * @param item The item to move into the queue.
     * @return `true` if the item was successfully pushed, `false` if the queue was full.
     * @note Contract: **Producer thread only.**
     */
    [[nodiscard]] bool try_push(T&& item) noexcept(std::is_nothrow_move_assignable_v<T>) {
        const auto current_tail = tail_.load(std::memory_order_relaxed);
        const auto next_tail = increment(current_tail);

        if (next_tail == head_.load(std::memory_order_acquire)) {
            return false; // Queue is full
        }

        buffer_[current_tail] = std::move(item);
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }

    /**
     * @brief Attempts to pop an item from the front of the queue.
     * @return A `Result` containing the item on success, or an `ErrorCode` if the queue is empty.
     * @note Contract: **Consumer thread only.**
     */
    [[nodiscard]] Result<T, ErrorCode> try_pop() noexcept(std::is_nothrow_move_constructible_v<T>) {
        const auto current_head = head_.load(std::memory_order_relaxed);
        if (current_head == tail_.load(std::memory_order_acquire)) {
            return Result<T, ErrorCode>::Failure(ErrorCode::IndexOutOfRange); // Queue is empty
        }

        T item = std::move(buffer_[current_head]);
        head_.store(increment(current_head), std::memory_order_release);
        return Result<T, ErrorCode>::Success(std::move(item));
    }

    /**
     * @brief Attempts to peek at the item at the front of the queue without removing it.
     * @return A `Result` containing a copy of the front item on success, or an `ErrorCode` if empty.
     * @note Contract: **Consumer thread only.**
     */
    [[nodiscard]] Result<T, ErrorCode> try_peek() const noexcept(std::is_nothrow_copy_constructible_v<T>) {
        const auto current_head = head_.load(std::memory_order_acquire);
        if (current_head == tail_.load(std::memory_order_acquire)) {
            return Result<T, ErrorCode>::Failure(ErrorCode::IndexOutOfRange); // Queue is empty
        }
        return Result<T, ErrorCode>::Success(buffer_[current_head]); // Returns a copy
    }

    /**
     * @brief Clears the queue, making it empty.
     * @note Safety: This operation is **NOT** thread-safe if called concurrently by both
     *        producer and consumer. It is safe if called by either thread while the other
     *        is known to be idle, or if called by a separate, synchronized "manager" thread.
     */
    void clear() noexcept {
        head_.store(0, std::memory_order_release);
        tail_.store(0, std::memory_order_release);
    }

    /**
     * @brief Checks if the queue is empty.
     * @return `true` if the queue is empty, `false` otherwise.
     * @note This method is safe to be called by both producer and consumer threads.
     */
    [[nodiscard]] bool empty() const noexcept {
        return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
    }

    /**
     * @brief Gets the current number of items in the queue.
     * @return The approximate number of items in the queue.
     * @note The returned size is approximate if called concurrently with push/pop operations.
     *       For an SPSC queue, if called by the consumer, it accurately reflects items available to pop.
     */
    [[nodiscard]] std::size_t size() const noexcept {
        const auto current_tail = tail_.load(std::memory_order_acquire);
        const auto current_head = head_.load(std::memory_order_acquire);
        return (current_tail + Capacity - current_head) & (Capacity - 1); // Bitwise modulo
    }

private:
    /**
     * @internal
     * @brief Increments an index, wrapping around the buffer capacity.
     * @param idx The index to increment.
     * @return The incremented and wrapped index.
     */
    [[nodiscard]] constexpr std::size_t increment(std::size_t idx) const noexcept {
        return (idx + 1) & (Capacity - 1);
    }

    // Align to cache line size to prevent false sharing between frequently accessed data.
    alignas(64) std::unique_ptr<T[]> buffer_;
    alignas(64) std::atomic<std::size_t> head_;
    alignas(64) std::atomic<std::size_t> tail_;
};

} // namespace RDT

#endif // TRAJECTORYQUEUE_H