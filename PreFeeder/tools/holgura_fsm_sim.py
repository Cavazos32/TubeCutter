#!/usr/bin/env python3
"""Simula la lógica holgura corregida (PreFeeder.ino) para validar tiempos de feeder."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


HOLG_ON_STABLE_MS = 50
HOLG_OFF_STABLE_MS = 100
SETTLE_MS = 150
ON_FEED_MS = 2000


class HolgPhase(IntEnum):
    IDLE = 0
    WAIT_ON = 1
    SETTLE = 2
    TIMED_FEED = 3


@dataclass
class State:
    holg_high_ms: int = 0
    holg_low_ms: int = 0
    holg_stable_on: bool = False
    feeder_demand: bool = False
    feeder_mode_holgura: bool = False
    phase: HolgPhase = HolgPhase.IDLE
    holg_last_on: bool = False
    holg_saw_on: bool = False
    post_cycle_watch: bool = False
    settle_end: int | None = None
    timed_end: int | None = None


def debounce(st: State, raw_on: bool, dt_ms: int) -> None:
    if raw_on:
        st.holg_high_ms += dt_ms
        st.holg_low_ms = 0
        if st.holg_high_ms >= HOLG_ON_STABLE_MS:
            st.holg_stable_on = True
    else:
        st.holg_low_ms += dt_ms
        st.holg_high_ms = 0
        if st.holg_low_ms >= HOLG_OFF_STABLE_MS:
            st.holg_stable_on = False


def begin_holgura(st: State) -> None:
    if st.holg_stable_on:
        return
    st.post_cycle_watch = False
    st.feeder_demand = False
    st.feeder_mode_holgura = True
    st.holg_saw_on = False
    st.settle_end = None
    st.timed_end = None
    st.holg_last_on = st.holg_stable_on
    st.phase = HolgPhase.WAIT_ON


def stop_cycle(st: State) -> None:
    st.feeder_demand = False
    st.feeder_mode_holgura = False
    st.phase = HolgPhase.IDLE
    st.timed_end = None
    if st.holg_saw_on:
        st.post_cycle_watch = True
        st.holg_saw_on = False


def service_holgura(st: State, now_ms: int) -> None:
    if not st.feeder_mode_holgura:
        return
    on = st.holg_stable_on
    rising = on and not st.holg_last_on
    st.holg_last_on = on

    if st.phase == HolgPhase.WAIT_ON:
        if rising:
            st.holg_saw_on = True
            st.phase = HolgPhase.SETTLE
            st.settle_end = now_ms + SETTLE_MS
            st.timed_end = None
        st.feeder_demand = not on
        return

    if st.phase == HolgPhase.SETTLE:
        st.feeder_demand = True
        if st.settle_end is not None and now_ms >= st.settle_end:
            st.phase = HolgPhase.TIMED_FEED
            st.timed_end = now_ms + ON_FEED_MS
        return

    if st.phase == HolgPhase.TIMED_FEED:
        st.feeder_demand = True
        if st.timed_end is not None and now_ms >= st.timed_end:
            stop_cycle(st)


def tick(st: State, now_ms: int, raw_on: bool, dt_ms: int = 10) -> None:
    debounce(st, raw_on, dt_ms)
    if st.post_cycle_watch and st.holg_stable_on:
        st.post_cycle_watch = False
    if (
        not st.feeder_demand
        and not st.holg_stable_on
        and not st.feeder_mode_holgura
    ):
        begin_holgura(st)
    service_holgura(st, now_ms)


def run_timeline(raw_fn, ms_total: int, dt_ms: int = 10) -> list[tuple[int, bool, HolgPhase, bool]]:
    st = State()
    log: list[tuple[int, bool, HolgPhase, bool]] = []
    for t in range(0, ms_total + dt_ms, dt_ms):
        raw = raw_fn(t)
        tick(st, t, raw, dt_ms)
        log.append((t, st.feeder_demand, st.phase, st.holg_stable_on))
    return log


def feed_ms_after_on(log: list, on_start_ms: int) -> int:
    """Ms de feeder_demand después de holgura estable ON (post flanco)."""
    fed = 0
    prev = False
    for t, demand, _phase, stable_on in log:
        if t < on_start_ms:
            prev = demand
            continue
        if not stable_on:
            prev = demand
            continue
        if demand and not prev:
            pass
        if demand:
            fed += 10
        prev = demand
    return fed


def assert_no_feed_while_stable_on_after_window(log: list, window_ms: int) -> None:
    """Tras window_ms desde primer stable ON, no debe haber feeder con holgura ON."""
    first_on: int | None = None
    for t, demand, _phase, stable_on in log:
        if stable_on and first_on is None:
            first_on = t
        if first_on is not None and t >= first_on + window_ms:
            if stable_on and demand:
                raise AssertionError(f"t={t} feeder activo con holgura estable ON tras ventana")


def main() -> None:
    # Ciclo normal: OFF 1s → ON el resto
    log = run_timeline(lambda t: t >= 1000, 8000)
    on_stable_at = next(t for t, _d, _p, s in log if s)
    post_on_feed = feed_ms_after_on(log, on_stable_at)
    expected = SETTLE_MS + ON_FEED_MS
    if not (expected - 30 <= post_on_feed <= expected + 80):
        raise AssertionError(
            f"post-ON feed={post_on_feed}ms, esperado ~{expected}ms (settle+on_s)"
        )
    assert_no_feed_while_stable_on_after_window(log, expected + 200)

    # Holgura ON todo el tiempo: auto no debe alimentar (sin ciclo previo OFF estable)
    st_on = State()
    st_on.holg_stable_on = True
    for t in range(0, 5000, 10):
        tick(st_on, t, True, 10)
    if st_on.feeder_demand:
        raise AssertionError("feeder activo con holgura ON desde t=0")

    # Parpadeo OFF 30ms con holgura física ON (raw sigue ON salvo glitch)
    def glitch(t: int) -> bool:
        if 3000 <= t < 3030:
            return False
        return t >= 1000

    log_g = run_timeline(glitch, 10000)
    assert_no_feed_while_stable_on_after_window(log_g, SETTLE_MS + ON_FEED_MS + 500)

    # Tras ciclo completo, holgura ON 3s extra: feeder quieto
    tail_demand = [d for t, d, ph, s in log if t >= 6000 and s]
    if any(tail_demand):
        raise AssertionError("feeder siguió tras completar timed con holgura ON")

    # Ciclo termina con holgura OFF (ON → settle → timed, luego OFF antes/despues del fin):
    # debe rear mar feeder, no quedar en post_cycle_watch sin alimentar.
    def on_then_off(t: int) -> bool:
        if t < 800:
            return False
        if t < 3500:
            return True
        return False

    log2 = run_timeline(on_then_off, 6000)
    # Tras holgura estable OFF otra vez (~3.5s en on_then_off), feeder debe volver ON.
    fed_after_reoff = any(
        d for t, d, _ph, s in log2 if t >= 3600 and not s
    )
    if not fed_after_reoff:
        raise AssertionError(
            "feeder no rearma tras ciclo holgura y holgura OFF de nuevo (post_cycle_watch)"
        )

    print("OK: simulación holgura — post-ON ~settle+on_s, sin arrastre con ON estable")


if __name__ == "__main__":
    main()
