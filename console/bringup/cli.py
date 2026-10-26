"""Command-line tools: the same core as the GUI, for scripts and CI.

    bringup monitor  --seconds 5             decoded bus traffic
    bringup params   --node 1 --set vel_kp=2 --save
    bringup test     --node 1 --node 2 --report-dir reports
    bringup record   --seconds 10 --out recordings
    bringup analyze  recordings/bus_....log
    bringup inject   --node 2 --kind overheat --value 60
"""

from __future__ import annotations

import argparse
import sys
import time

from . import connect
from .bringup_test import BringupTest
from .params import PARAMS, ParamError
from .protocol import InjectKind, fault_text
from .recorder import Recorder, analyze_log
from .transport import LinkError

DEFAULT_URL = "tcp://localhost:29536"


def _connect(args):
    link, dm, pc = connect(args.url)
    if not dm.wait_until(lambda: len(dm.node_ids()) > 0, 2.0):
        print(f"connected to {args.url} but no actuator is talking", file=sys.stderr)
    return link, dm, pc


def cmd_monitor(args) -> int:
    link, dm, _ = _connect(args)
    seen_ids = set()

    def show(frame, direction, decoded):
        if args.only_new and frame.can_id in seen_ids:
            return
        seen_ids.add(frame.can_id)
        name = dm.codec.name_of(frame.can_id)
        sig = ""
        if decoded:
            sig = " ".join(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}" for k, v in decoded.signals.items())
        print(f"{frame.timestamp:.3f} {direction} {frame.can_id:03X} [{len(frame.data)}] {frame.data.hex().upper():<16} "
              f"{name:<16} {sig}")

    dm.add_listener(show)
    try:
        time.sleep(args.seconds)
    finally:
        dm.remove_listener(show)
        stats = dm.bus_stats()
        print(f"\n{stats['frames_per_s']:.0f} frames/s, bus load {stats['bus_load_pct']:.1f}% of 500 kbit/s")
        dm.stop()
        link.close()
    return 0


def cmd_params(args) -> int:
    link, dm, pc = _connect(args)
    try:
        if args.load_defaults:
            pc.load_defaults(args.node)
            print(f"node {args.node}: defaults loaded")
        if args.import_yaml:
            written = pc.import_yaml(args.node, args.import_yaml)
            print(f"node {args.node}: wrote {len(written)} parameters from {args.import_yaml}")
        for item in args.set or []:
            name, _, value = item.partition("=")
            stored = pc.write(args.node, name.strip(), float(value))
            print(f"node {args.node}: {name.strip()} = {stored:g}")
        if args.save:
            pc.save(args.node)
            print(f"node {args.node}: saved to flash")
        if args.export_yaml:
            pc.export_yaml(args.node, args.export_yaml)
            print(f"node {args.node}: exported to {args.export_yaml}")
        values = pc.read_all(args.node)
        print(f"\n{'#':>2}  {'name':<16} {'value':>12}  unit")
        for p in PARAMS:
            flag = " (ro)" if p.read_only else ""
            print(f"{p.index:>2}  {p.name:<16} {values[p.name]:>12.5g}  {p.unit}{flag}")
    except ParamError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    finally:
        dm.stop()
        link.close()
    return 0


def cmd_test(args) -> int:
    link, dm, pc = _connect(args)
    nodes = args.node or dm.node_ids()
    if not nodes:
        print("no actuators found", file=sys.stderr)
        return 2
    reports = []
    try:
        for n in nodes:
            report = BringupTest(dm, pc, n, fix=not args.no_fix, log=print).run()
            reports.append(report)
            md, js = report.save(args.report_dir)
            print(report.to_markdown())
            print(f"report: {md}\n        {js}\n")
    finally:
        dm.stop()
        link.close()
    ok = all(r.passed for r in reports)
    print("ALL PASS" if ok else "SOME CHECKS FAILED")
    return 0 if ok else 1


def cmd_record(args) -> int:
    link, dm, _ = _connect(args)
    rec = Recorder(dm)
    log_path, csv_path = rec.start(args.out)
    print(f"recording {args.seconds:g} s to {log_path} and {csv_path} ...")
    try:
        time.sleep(args.seconds)
    finally:
        rec.stop()
        dm.stop()
        link.close()
    print(f"{rec.frames} frames, {rec.rows} CSV rows")
    return 0


def cmd_analyze(args) -> int:
    summary = analyze_log(args.log)
    print(summary.to_text())
    return 0


def cmd_inject(args) -> int:
    link, dm, _ = _connect(args)
    try:
        kind = InjectKind[args.kind.upper()]
        dm.inject(args.node, kind, args.value)
        time.sleep(0.2)
        print(f"injected {kind.name} ({args.value:g}) on node {args.node}; "
              f"faults now: {fault_text(dm.node(args.node).fault_flags)}")
    finally:
        dm.stop()
        link.close()
    return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="bringup", description="Actuator bring-up and diagnostics tools")
    ap.add_argument("--url", default=DEFAULT_URL, help=f"SLCAN link (default {DEFAULT_URL})")
    sub = ap.add_subparsers(dest="command", required=True)

    p = sub.add_parser("monitor", help="print decoded bus traffic")
    p.add_argument("--seconds", type=float, default=3.0)
    p.add_argument("--only-new", action="store_true", help="show each ID once")
    p.set_defaults(fn=cmd_monitor)

    p = sub.add_parser("params", help="read, write, save, import/export parameters")
    p.add_argument("--node", type=int, default=1)
    p.add_argument("--set", action="append", metavar="NAME=VALUE")
    p.add_argument("--save", action="store_true", help="save to flash")
    p.add_argument("--load-defaults", action="store_true")
    p.add_argument("--export-yaml", metavar="FILE")
    p.add_argument("--import-yaml", metavar="FILE")
    p.set_defaults(fn=cmd_params)

    p = sub.add_parser("test", help="run the automated bring-up test")
    p.add_argument("--node", type=int, action="append", help="node(s) to test (default: all found)")
    p.add_argument("--no-fix", action="store_true", help="report problems without fixing them")
    p.add_argument("--report-dir", default="reports")
    p.set_defaults(fn=cmd_test)

    p = sub.add_parser("record", help="record the bus to candump .log + decoded .csv")
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--out", default="recordings")
    p.set_defaults(fn=cmd_record)

    p = sub.add_parser("analyze", help="summarise a recorded candump log")
    p.add_argument("log")
    p.set_defaults(fn=cmd_analyze)

    p = sub.add_parser("inject", help="simulator fault injection")
    p.add_argument("--node", type=int, default=1)
    p.add_argument("--kind", required=True, choices=[k.name.lower() for k in InjectKind])
    p.add_argument("--value", type=float, default=0.0)
    p.set_defaults(fn=cmd_inject)
    return ap


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.fn(args)
    except LinkError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
