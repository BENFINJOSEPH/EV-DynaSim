"""Command line interface.

evdynasim steady    --config configs/generic_ev.yaml --speed 50 100 130
evdynasim cycle     --config configs/generic_ev.yaml --cycle nedc --out results/nedc
evdynasim cycle     --config configs/generic_ev.yaml --cycle nedc --engine cpp
evdynasim forward   --config configs/generic_ev.yaml --cycle nedc --out results/forward
evdynasim validate  --config X.yaml --drives a.csv b.csv c.csv --mapping m.yaml --n-train 2 --out results/val
evdynasim demo-validation --config configs/generic_ev.yaml --out results/validation
evdynasim study     sensitivity|ambient --config configs/generic_ev.yaml --out results/studies
evdynasim benchmark --config configs/generic_ev.yaml
evdynasim all       --config configs/generic_ev.yaml --out results     (everything above)
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from evdynasim.cycles import get_cycle, nedc
from evdynasim.params import load_params, with_changes
from evdynasim.simulate import simulate_cycle, steady_state_power_w


def _out(path: str | None) -> Path | None:
    if not path:
        return None
    out = Path(path)
    out.mkdir(parents=True, exist_ok=True)
    return out


def _print_dict(d: dict) -> None:
    for key, value in d.items():
        print(f"  {key:<30} {value:12.4f}" if isinstance(value, float) else f"  {key:<30} {value}")


# ---- sub-commands ----------------------------------------------------------------
def cmd_steady(args) -> int:
    p = load_params(args.config)
    for v in args.speed:
        power_kw = float(steady_state_power_w(p, v)) / 1e3
        cons = power_kw / v * 100.0 if v > 0 else float("nan")
        print(f"{v:6.1f} km/h  ->  {power_kw:7.2f} kW  ({cons:5.2f} kWh/100 km)")
    if out := _out(args.out):
        from evdynasim.plotting import plot_steady_state

        print(f"plot: {plot_steady_state(p, out / 'steady_state.png')}")
    return 0


def cmd_cycle(args) -> int:
    p = load_params(args.config)
    cycle = get_cycle(args.cycle)
    print(f"Vehicle : {p.name}\nCycle   : {cycle.name}\nEngine  : {args.engine}")
    if args.engine == "cpp":
        from evdynasim import cpp_engine

        _print_dict(cpp_engine.simulate_cpp(args.config, cycle)["summary"])
        return 0

    res = simulate_cycle(p, cycle)
    _print_dict(res.summary)
    if out := _out(args.out):
        from evdynasim.plotting import plot_cycle, plot_energy_breakdown, plot_motor_map

        stem = cycle.name.lower().replace(" ", "_")
        res.to_dataframe().to_csv(out / f"{stem}_timeseries.csv", index=False)
        (out / f"{stem}_summary.json").write_text(json.dumps(res.summary, indent=2))
        plot_cycle(res, out / f"{stem}_cycle.png")
        plot_energy_breakdown(res, out / f"{stem}_energy.png")
        if p.motor.loss_map is not None:
            plot_motor_map(p, res, out / f"{stem}_motor_map.png")
        print(f"results written to {out.resolve()}")
    return 0


def cmd_forward(args) -> int:
    from evdynasim.forward import DriverConfig, simulate_forward

    p = load_params(args.config)
    if args.max_power_kw:
        p = with_changes(p, {"motor.max_power_w": args.max_power_kw * 1e3})
    cycle = get_cycle(args.cycle)
    cfg = DriverConfig(anti_windup=not args.no_anti_windup, integrator=args.integrator)
    r = simulate_forward(p, cycle, cfg)
    print(f"Forward simulation: {cycle.name}, anti-windup={cfg.anti_windup}, {cfg.integrator}")
    _print_dict(r.summary)
    if out := _out(args.out):
        from evdynasim.plotting import plot_forward

        plot_forward({"forward": r}, out / "forward.png", window=(0, cycle.duration_s),
                     title=f"Forward simulation - {cycle.name}")  # fmt: skip
        print(f"plot written to {out / 'forward.png'}")
    return 0


def cmd_charge(args) -> int:
    from evdynasim.charging import simulate_charging

    p = load_params(args.config)
    r = simulate_charging(p, args.soc_start, args.soc_target, ambient_temp_c=args.ambient,
                          initial_temp_c=args.battery_temp, charger_power_kw=args.charger_kw)  # fmt: skip
    print("DC charging session")
    _print_dict(r.summary)
    if out := _out(args.out):
        from evdynasim.plotting import plot_charging_session

        r.series.to_csv(out / "charging_timeseries.csv", index=False)
        (out / "charging_summary.json").write_text(json.dumps(r.summary, indent=2))
        plot_charging_session(r, out / "charging_session.png")
        print(f"results written to {out.resolve()}")
    return 0


def cmd_validate(args) -> int:
    from evdynasim.measured import load_mapping, load_measured_csv
    from evdynasim.report import run_validation

    p = load_params(args.config)
    mapping = load_mapping(args.mapping)
    drives = [load_measured_csv(f, mapping, smooth_window=args.smooth) for f in args.drives]
    cal, _, after = run_validation(p, Path(args.config), drives, Path(args.out), args.n_train)
    print(cal.table.to_string(index=False))
    print(after[["drive", "rmse_kw", "energy_error_pct"]].to_string(index=False))
    print(f"report: {Path(args.out) / 'report.md'}")
    return 0


def cmd_demo_validation(args) -> int:
    from evdynasim.report import run_synthetic_validation

    p = load_params(args.config)
    results = run_synthetic_validation(p, Path(args.config), Path(args.out))
    for name, (cal, before, after) in results.items():
        print(f"\n{name}")
        print(cal.table[["parameter", "estimate", "ci95_low", "ci95_high"]].to_string(index=False))
        print(f"  validation |energy error|: {before.energy_error_pct.abs().mean():.2f} % -> "
              f"{after.energy_error_pct.abs().mean():.2f} %")  # fmt: skip
    print(f"\nreports in {Path(args.out).resolve()}")
    return 0


def cmd_study(args) -> int:
    from evdynasim import plotting, studies

    p = load_params(args.config)
    out = _out(args.out)
    if args.kind == "sensitivity":
        tables = {
            "NEDC": studies.sensitivity(p, nedc()),
            "Motorway 120 km/h": studies.sensitivity(p, studies.motorway_cycle(120.0)),
        }
        for name, df in tables.items():
            print(f"\n{name} (base {df['base'].iloc[0]:.2f} kWh/100 km)")
            print(df[["parameter", "minus_pct", "plus_pct"]].round(2).to_string(index=False))
            if out:
                df.to_csv(out / f"sensitivity_{name.split()[0].lower()}.csv", index=False)
        if out:
            plotting.plot_tornado(tables, out / "sensitivity_tornado.png")
    elif args.kind == "charging":
        from evdynasim import charging

        amb = charging.ambient_study(p)
        pw = charging.charger_power_study(p)
        print(amb.round(1).to_string(index=False))
        print(pw.round(1).to_string(index=False))
        if out:
            amb.to_csv(out / "charging_ambient.csv", index=False)
            pw.to_csv(out / "charging_power.csv", index=False)
            plotting.plot_charging_studies(amb, pw, out / "charging_studies.png")
            runs = {
                "25 degC": charging.simulate_charging(p, 0.1, 0.8, ambient_temp_c=25.0),
                "10 degC parked": charging.simulate_charging(p, 0.1, 0.8, ambient_temp_c=10.0),
                "0 degC parked": charging.simulate_charging(p, 0.1, 0.8, ambient_temp_c=0.0),
                "0 degC preconditioned": charging.simulate_charging(
                    p, 0.1, 0.8, ambient_temp_c=0.0, initial_temp_c=25.0
                ),
            }
            plotting.plot_charging_curves(runs, out / "charging_curves.png")
            full = charging.simulate_charging(p, 0.1, 1.0)
            plotting.plot_charging_session(full, out / "charging_session_full.png")
            plotting.plot_charging_session(runs["25 degC"], out / "charging_session_10_80.png")
    else:
        df = studies.ambient_sweep(p, nedc())
        print(df.round(2).to_string(index=False))
        if out:
            df.to_csv(out / "ambient_sweep.csv", index=False)
            plotting.plot_ambient(df, "NEDC", out / "ambient_sweep.png")
    return 0


def cmd_benchmark(args) -> int:
    from evdynasim import studies

    p = load_params(args.config)
    df = studies.benchmark(p, Path(args.config), nedc(), args.repeat)
    print(df.round(3).to_string(index=False))
    return 0


def cmd_all(args) -> int:
    """Regenerate every result and figure used in the documentation."""
    from evdynasim import plotting, studies
    from evdynasim.forward import DriverConfig, simulate_forward

    out = Path(args.out)
    cfg = args.config
    cmd_steady(argparse.Namespace(config=cfg, speed=[50.0, 100.0, 130.0], out=str(out / "steady")))
    cmd_cycle(argparse.Namespace(config=cfg, cycle="nedc", engine="python", out=str(out / "nedc")))
    cmd_study(argparse.Namespace(config=cfg, kind="sensitivity", out=str(out / "studies")))
    cmd_study(argparse.Namespace(config=cfg, kind="ambient", out=str(out / "studies")))
    cmd_study(argparse.Namespace(config=cfg, kind="charging", out=str(out / "charging")))

    p = load_params(cfg)
    weak = with_changes(p, {"motor.max_power_w": 25_000.0})
    runs = {
        "PI with anti-windup": simulate_forward(weak, nedc(), DriverConfig(anti_windup=True)),
        "PI without anti-windup": simulate_forward(weak, nedc(), DriverConfig(anti_windup=False)),
    }
    (out / "forward").mkdir(parents=True, exist_ok=True)
    plotting.plot_forward(runs, out / "forward" / "anti_windup.png")
    for label, r in runs.items():
        print(f"{label}: overshoot {r.summary['max_overshoot_kmh']:.2f} km/h")

    cmd_demo_validation(argparse.Namespace(config=cfg, out=str(out / "validation")))
    print(studies.benchmark(p, Path(cfg), nedc(), 10).round(3).to_string(index=False))
    return 0


# ---- parser ------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="evdynasim", description="EV energy simulation platform")
    sub = parser.add_subparsers(dest="command", required=True)

    def add(name, func, help_text):
        sp = sub.add_parser(name, help=help_text)
        sp.add_argument("--config", required=True, help="vehicle YAML file")
        sp.set_defaults(func=func)
        return sp

    s = add("steady", cmd_steady, "steady-state power at constant speed")
    s.add_argument("--speed", type=float, nargs="+", default=[50.0, 100.0, 130.0], help="km/h")
    s.add_argument("--out", help="folder for the plot")

    c = add("cycle", cmd_cycle, "backward simulation of a driving cycle")
    c.add_argument(
        "--cycle", default="nedc", help="'nedc' or a CSV file (time_s, speed_kmh[, grade_pct])"
    )
    c.add_argument("--engine", choices=["python", "cpp"], default="python")
    c.add_argument("--out", help="folder for CSV, JSON and plots")

    f = add("forward", cmd_forward, "forward simulation with a PI driver")
    f.add_argument("--cycle", default="nedc")
    f.add_argument("--max-power-kw", type=float, help="override the motor power")
    f.add_argument("--no-anti-windup", action="store_true")
    f.add_argument("--integrator", choices=["rk4", "euler"], default="rk4")
    f.add_argument("--out")

    ch = add("charge", cmd_charge, "simulate a DC fast-charging session")
    ch.add_argument("--soc-start", type=float, default=0.10)
    ch.add_argument("--soc-target", type=float, default=0.80)
    ch.add_argument("--charger-kw", type=float, help="charging station power (default: config)")
    ch.add_argument("--ambient", type=float, help="ambient temperature in degC (default: config)")
    ch.add_argument(
        "--battery-temp", type=float, help="battery temperature at plug-in (default: ambient)"
    )
    ch.add_argument("--out")

    v = add("validate", cmd_validate, "calibrate on measured drives and validate on others")
    v.add_argument(
        "--drives", nargs="+", required=True, help="CSV files; first n-train are for calibration"
    )
    v.add_argument("--mapping", help="signal mapping YAML (default: canonical column names)")
    v.add_argument("--n-train", type=int, required=True)
    v.add_argument("--smooth", type=int, default=1, help="speed moving-average window (odd)")
    v.add_argument("--out", required=True)

    d = add("demo-validation", cmd_demo_validation, "calibration + validation on synthetic data")
    d.add_argument("--out", default="results/validation")

    st = add("study", cmd_study, "sensitivity or ambient temperature study")
    st.add_argument("kind", choices=["sensitivity", "ambient", "charging"])
    st.add_argument("--out")

    b = add("benchmark", cmd_benchmark, "runtime of the Python and C++ engines")
    b.add_argument("--repeat", type=int, default=20)

    a = add("all", cmd_all, "regenerate all results and figures")
    a.add_argument("--out", default="results")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
