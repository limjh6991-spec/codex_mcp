"""Command-line entry point for the RoArm application launcher."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import Iterable, Optional, Sequence

from .launch import envvars, overlay, extensions
from .runtime import run_loop


_HEADLESS_ENABLE_ONLY: tuple[str, ...] = (
    "omni.physx",
    "omni.isaac.core",
    "omni.isaac.gym",
    "omni.isaac.lab",
    "omni.usd",
    "omni.usdphysics",
    "omni.kit.loop-isaac",
    "omni.timeline",
)

_HEADLESS_DISABLE_PATTERNS: tuple[str, ...] = (
    "omni.kit.hotkeys.core",
    "omni.kit.window.*",
    "omni.kit.viewport.*",
    "omni.kit.menu.*",
    "omni.kit.widget.*",
    "omni.kit.browser.*",
    "omni.kit.ui*",
    "omni.kit.manipulator.*",
    "isaacsim.gui.*",
    "isaacsim.app.about",
    "isaacsim.asset.browser",
    "omni.replicator.*",
    "isaacsim.replicator.*",
    "omni.syntheticdata",
    "omni.syntheticdata.asynchronous",
    "omni.syntheticdata.ui",
    "omni.services.replicator",
    "omni.services.replicator.commands",
    "omni.services.replicator.scripts",
    "omni.graph.replicator",
    "carb.audio",
    "omni.audio.*",
    "omni.uiaudio",
    "omni.kit.property.audio",
    "omni.isaac.audio",
)


def _extend_sys_argv(extra: Iterable[str]) -> None:
    """Append unique Kit arguments to ``sys.argv`` for SimulationApp consumption."""

    for item in extra:
        if item not in sys.argv:
            sys.argv.append(item)


def _apply_headless_kit_guards(
    *,
    experience_override: Optional[str],
    run_notes: list[str],
    extra_disable: Iterable[str] = (),
) -> None:
    """Inject ``--enableOnly`` and explicit disable flags for headless safety."""

    if "--enableOnly" not in sys.argv:
        enable_only_arg = ",".join(_HEADLESS_ENABLE_ONLY)
        _extend_sys_argv(["--enableOnly", enable_only_arg])
        run_notes.append(f"kit_enable_only={len(_HEADLESS_ENABLE_ONLY)}")

    disable_targets = list(_HEADLESS_DISABLE_PATTERNS) + [ext for ext in extra_disable if ext]
    disable_flags = [
        f"--/exts/{module_name}/enabled=false"
        for module_name in disable_targets
        if module_name and "*" not in module_name
    ]
    missing_flags = [flag for flag in disable_flags if flag not in sys.argv]
    if missing_flags:
        _extend_sys_argv(missing_flags)
        run_notes.append(f"kit_disable={len(disable_flags)}")

    if experience_override and "--overrideApp" not in sys.argv:
        _extend_sys_argv(["--overrideApp", experience_override])
        run_notes.append("override_app=forced")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Parse CLI arguments for the launcher."""

    parser = argparse.ArgumentParser(
        description="Launch Isaac Sim GUI and load a USD stage (defaults to RoArm M3)."
    )
    parser.add_argument(
        "usd",
        nargs="?",
        default="assets/roarm_m3/usd/roarm_m3.generated.usd",
        help="불러올 USD 스테이지 경로 (기본값: assets/roarm_m3/usd/roarm_m3.generated.usd).",
    )
    parser.add_argument(
        "--prim",
        default="/World/roarm_m3",
        help="Prim path to focus after loading (default: /World/roarm_m3).",
    )
    parser.add_argument(
        "--mode",
        choices=("train", "review"),
        default="train",
        help="train: headless 안정 모드, review: GUI 디버깅 모드 (기본값: train).",
    )
    parser.add_argument(
        "--no-quiet-shutdown",
        action="store_true",
        help="Disable CARB quiet shutdown to keep terminals open and surface full logs.",
    )
    parser.add_argument(
        "--no-blocklist",
        action="store_true",
        help="Disable headless extension blocklist and overlay tweaks for A/B testing.",
    )
    parser.add_argument(
        "--warmup-steps",
        type=int,
        default=0,
        help="Simulation steps to run after loading for stage stabilization.",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=0,
        help="train 모드에서 추가로 실행할 시뮬레이션 스텝 수 (0이면 무한 루프).",
    )
    parser.add_argument(
        "--trajectory-log",
        type=Path,
        help="prim 트래젝토리를 JSON으로 기록할 파일 경로.",
    )
    parser.add_argument(
        "--usd-export",
        type=Path,
        help="종료 시 현재 스테이지를 지정한 USD 파일로 Export.",
    )
    parser.add_argument(
        "--debug-trace",
        action="store_true",
        help="런처 실행 경로를 상세 로그로 출력하여 조기 종료 지점을 추적합니다.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Execute the CLI entry point."""

    args = parse_args(argv)
    usd_path = Path(args.usd).expanduser().resolve()
    if not usd_path.exists():
        raise SystemExit(f"USD 파일을 찾을 수 없습니다: {usd_path}")

    headless_mode = args.mode == "train"
    run_notes: list[str] = []

    run_notes.append(envvars.configure_quiet_shutdown(args.no_quiet_shutdown))
    run_notes.extend(envvars.apply_headless_defaults(headless_mode))

    replicator_enabled = os.environ.get("ROARM_ENABLE_REPLICATOR")
    replicator_should_disable = not (
        replicator_enabled and replicator_enabled.lower() in {"1", "true", "yes", "on"}
    )

    blocklist_enabled = headless_mode and not args.no_blocklist
    applied_blocklist: set[str] = set()
    if blocklist_enabled:
        applied_blocklist = extensions.collect_extension_blocklist(replicator_should_disable)
        disabled_exts = set(filter(None, os.environ.get("OMNI_KIT_DISABLE_EXTENSIONS", "").split(",")))
        disabled_exts.update(applied_blocklist)
        disabled_exts.update(_HEADLESS_DISABLE_PATTERNS)
        os.environ["OMNI_KIT_DISABLE_EXTENSIONS"] = ",".join(sorted(disabled_exts))
        os.environ.setdefault("CARB_AUDIO_DISABLED", "1")
        run_notes.append(f"blocklist_size={len(applied_blocklist)}")
        print(
            "[open_roarm_m3_gui] headless mode disabled extensions: "
            f"{os.environ['OMNI_KIT_DISABLE_EXTENSIONS']}"
        )
        blocklist_flags = [
            f"--/exts/{ext}/enabled=false"
            for ext in applied_blocklist
            if ext and "*" not in ext
        ]
        blocklist_missing = [flag for flag in blocklist_flags if flag not in sys.argv]
        if blocklist_missing:
            _extend_sys_argv(blocklist_missing)
            run_notes.append(f"kit_disable_blocklist={len(blocklist_flags)}")
    elif headless_mode:
        run_notes.append("blocklist=disabled")

    experience_override: Optional[str] = None
    if headless_mode:
        overlay_events: list[str] = []

        def overlay_logger(event: str, payload: dict[str, object]) -> None:
            detail = payload.get("reason") or payload.get("target") or ""
            overlay_events.append(f"{event}:{detail}" if detail else event)

        overlay_env_disabled = bool(os.environ.get("ROARM_DISABLE_HEADLESS_OVERLAY"))
        overlay_result = overlay.prepare_headless_overlay(
            skip_env=overlay_env_disabled,
            skip_cli=args.no_blocklist,
            logger=overlay_logger,
        )
        if overlay_result.note:
            run_notes.append(overlay_result.note)

        experience_override = overlay_result.experience

        if overlay_result.status == "skipped":
            reason = overlay_result.reason or "unknown"
            print(
                "[open_roarm_m3_gui] 정보: headless overlay 적용이 비활성화되었습니다"
                f" (reason={reason})."
            )
        elif overlay_result.status == "failed_copy":
            print(
                "[open_roarm_m3_gui] 경고: headless overlay를 Isaac Sim apps 디렉터리에 복사하지 못해"
                " 기본 experience로 실행합니다."
            )
        elif overlay_result.status == "missing_source":
            source = overlay_result.source or overlay.overlay_source()
            print(
                f"[open_roarm_m3_gui] 경고: headless overlay 원본({source})을 찾을 수 없어 기본 experience로 실행합니다."
            )
        elif overlay_result.status == "missing_root":
            print(
                "[open_roarm_m3_gui] 경고: Isaac Sim 설치 경로를 찾을 수 없어 기본 experience로 실행합니다."
            )

        for event_entry in overlay_events:
            if not event_entry.startswith("overlay_applied"):
                run_notes.append(f"overlay_event={event_entry}")

        _apply_headless_kit_guards(
            experience_override=experience_override,
            run_notes=run_notes,
            extra_disable=applied_blocklist if blocklist_enabled else (),
        )
    elif headless_mode:
        _apply_headless_kit_guards(
            experience_override=None,
            run_notes=run_notes,
            extra_disable=applied_blocklist if blocklist_enabled else (),
        )

    if args.debug_trace:
        run_notes.append("debug_trace=enabled")

    _exit_reason = run_loop.launch_simulation(
        args=args,
        usd_path=usd_path,
        headless_mode=headless_mode,
        blocklist_enabled=blocklist_enabled,
        applied_blocklist=applied_blocklist,
        experience_override=experience_override,
        run_notes=run_notes,
        log_prefix="open_roarm_m3_gui",
        debug_trace=args.debug_trace,
    )

    return 0
