#!/usr/bin/env python3
"""Collect the detector's self-reported rate and latency for a fixed window, then
assert thresholds. Exit code is the verdict, so shell harnesses can chain on it."""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate-topic", required=True)
    ap.add_argument("--latency-topic", required=True)
    ap.add_argument("--min-rate", type=float, required=True)
    ap.add_argument("--max-latency", type=float, required=True)
    ap.add_argument("--duration", type=float, default=15.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node("replay_metrics_collector")
    rates, latencies = [], []
    node.create_subscription(Float32, args.rate_topic, lambda m: rates.append(m.data), 10)
    node.create_subscription(
        Float32, args.latency_topic, lambda m: latencies.append(m.data), 10)

    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if not rates or not latencies:
        print(f"FAIL: no reports received in {args.duration:.0f}s "
              f"(rates={len(rates)}, latencies={len(latencies)}) -- is the detector up?")
        return 1

    best_rate = max(rates)              # reports include spin-up; judge the steady state
    best_latency = min(latencies)
    print(f"rate samples    : {[round(r, 1) for r in rates]}  (best {best_rate:.1f} Hz)")
    print(f"latency samples : {[round(v, 1) for v in latencies]}  (best {best_latency:.1f} ms)")

    ok = best_rate >= args.min_rate and best_latency <= args.max_latency
    print(f"thresholds      : rate >= {args.min_rate} Hz, latency <= {args.max_latency} ms")
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
