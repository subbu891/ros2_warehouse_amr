from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Protocol


class StepKind(str, Enum):
    GOTO = "goto"
    GOTO_POSE = "goto_pose"
    WAIT = "wait"
    DOCK = "dock"
    UNDOCK = "undock"


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Step:
    kind: StepKind
    raw: str
    pose: Pose2D | None = None
    seconds: float = 0.0
    target: str = ""


class MissionError(ValueError):
    pass


# Parse one step string such as "goto:pickup_a" or "goto_pose:1.0,2.0,0.0"
def parse_step(raw: str, stations: dict[str, Pose2D]) -> Step:
    text = raw.strip()
    kind, _, arg = text.partition(":")
    kind = kind.strip().lower()
    arg = arg.strip()
    if kind == StepKind.GOTO:
        if arg not in stations:
            raise MissionError(f"unknown station '{arg}' (known: {sorted(stations)})")
        return Step(StepKind.GOTO, text, pose=stations[arg], target=arg)
    if kind == StepKind.GOTO_POSE:
        try:
            x, y, yaw = (float(v) for v in arg.split(","))
        except ValueError as exc:
            raise MissionError(f"goto_pose needs 'x,y,yaw', got '{arg}'") from exc
        return Step(StepKind.GOTO_POSE, text, pose=Pose2D(x, y, yaw))
    if kind == StepKind.WAIT:
        try:
            sec = float(arg)
        except ValueError as exc:
            raise MissionError(f"wait needs seconds, got '{arg}'") from exc
        if sec < 0:
            raise MissionError("wait must be >= 0")
        return Step(StepKind.WAIT, text, seconds=sec)
    if kind == StepKind.DOCK:
        if not arg:
            raise MissionError("dock needs a dock id")
        return Step(StepKind.DOCK, text, target=arg)
    if kind == StepKind.UNDOCK:
        return Step(StepKind.UNDOCK, text)
    raise MissionError(f"unknown step '{raw}'")


# Parse a full mission and reject plans that would drive while docked
def parse_mission(steps: list[str], stations: dict[str, Pose2D]) -> list[Step]:
    if not steps:
        raise MissionError("mission has no steps")
    parsed = [parse_step(s, stations) for s in steps]
    docked = False
    for s in parsed:
        if s.kind == StepKind.DOCK:
            docked = True
        elif s.kind == StepKind.UNDOCK:
            docked = False
        elif s.kind in (StepKind.GOTO, StepKind.GOTO_POSE) and docked:
            raise MissionError(f"'{s.raw}' while docked: add 'undock' first")
    return parsed


def load_stations(data: dict) -> dict[str, Pose2D]:
    return {name: Pose2D(float(v["x"]), float(v["y"]), float(v["yaw"]))
            for name, v in data["stations"].items()}


class Outcome(str, Enum):
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


# Interface to the robot; the ROS node implements it with Nav2, the tests with a fake
class Executor(Protocol):
    def navigate(self, pose: Pose2D) -> Outcome: ...
    def dock(self, dock_id: str) -> Outcome: ...
    def undock(self) -> Outcome: ...
    def wait(self, seconds: float) -> Outcome: ...
    def recover(self, attempt: int) -> None: ...


# Attempts per step including the first try
@dataclass
class RetryPolicy:
    max_attempts: int = 3


@dataclass
class MissionResult:
    success: bool
    message: str
    failed_step: int = -1
    duration_s: float = 0.0
    attempts: list[int] = field(default_factory=list)


Feedback = Callable[[int, "Step", str, int], None]


# Runs the steps in order; a failed step triggers recovery and a retry until max_attempts
class MissionRunner:
    def __init__(self, executor: Executor, policy: RetryPolicy | None = None,
                 clock: Callable[[], float] = time.monotonic):
        self.ex = executor
        self.policy = policy or RetryPolicy()
        self.clock = clock
        self._cancel = False

    def cancel(self) -> None:
        self._cancel = True

    def _run_step(self, step: Step) -> Outcome:
        if step.kind in (StepKind.GOTO, StepKind.GOTO_POSE):
            return self.ex.navigate(step.pose)
        if step.kind == StepKind.WAIT:
            return self.ex.wait(step.seconds)
        if step.kind == StepKind.DOCK:
            return self.ex.dock(step.target)
        return self.ex.undock()

    def run(self, steps: list[Step], feedback: Feedback | None = None) -> MissionResult:
        start = self.clock()
        attempts_log = []
        for i, step in enumerate(steps):
            attempt = 1
            while True:
                if self._cancel:
                    return MissionResult(False, "canceled", i, self.clock() - start, attempts_log)
                if feedback:
                    feedback(i, step, "RUNNING" if attempt == 1 else "RETRYING", attempt)
                outcome = self._run_step(step)
                if outcome == Outcome.SUCCEEDED:
                    attempts_log.append(attempt)
                    break
                if outcome == Outcome.CANCELED or self._cancel:
                    return MissionResult(False, "canceled", i, self.clock() - start, attempts_log)

                # A wait cannot be retried meaningfully; every other step gets recovery and a retry
                if step.kind == StepKind.WAIT or attempt >= self.policy.max_attempts:
                    attempts_log.append(attempt)
                    return MissionResult(False, f"step {i} '{step.raw}' failed after {attempt} attempt(s)",
                                         i, self.clock() - start, attempts_log)
                if feedback:
                    feedback(i, step, "RECOVERING", attempt)
                self.ex.recover(attempt)
                attempt += 1
        return MissionResult(True, "mission complete", -1, self.clock() - start, attempts_log)
