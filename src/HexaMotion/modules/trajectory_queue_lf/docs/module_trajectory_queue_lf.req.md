# Requirements for Module: `trajectory_queue_lf`

## 1. Functional Requirements

- **[!REQ] REQ-QUEUE-01: Lock-Free Single-Producer/Single-Consumer Queue**  
  The module must provide a wait-free or lock-free queue optimized for one producer (planner) and one consumer (motion manager).

- **[!REQ] REQ-QUEUE-02: Bounded Capacity**  
  The queue must have a fixed maximum size to guarantee deterministic memory usage in RT contexts.

- **[!REQ] REQ-QUEUE-03: Non-Blocking Operations**  
  `enqueue` and `dequeue` must never block or allocate memory.

---

## 2. Non-Functional Requirements (NFRs)

- **[!NFR] NFR-QUEUE-01: RT-Safe**  
  All operations must be safe to call from RT threads without mutexes.

- **[!NFR] NFR-QUEUE-02: Deterministic Latency**  
  Each operation must complete in O(1) time with predictable latency.

---

## 3. Interfaces

- `TrajectoryQueue::push(item)`
- `TrajectoryQueue::pop(item)`
- `TrajectoryQueue::size()`
- `TrajectoryQueue::capacity()`

---

## 4. Architecture Summary

- Implements a bounded SPSC ring buffer used between planner and motion manager.
- All memory is pre-allocated; indexes are atomics with relaxed ordering where safe.
- Designed for high-throughput RT queues (no heap, no locks).

## 5. Test Recommendations

**Unit Tests**
- Push/pop preserves FIFO order under single producer/consumer.
- Overwrite/full detection returns expected error code.

**Integration Tests**
- Planner → MotionManager queue fill/empty behavior at RT buffer thresholds.