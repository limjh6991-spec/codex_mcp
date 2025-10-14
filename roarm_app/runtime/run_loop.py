"""Runtime utilities for launching the RoArm Isaac Sim experience."""

from __future__ import annotations

import asyncio
import gc
import importlib
import json
import os
import sys
import time
from argparse import Namespace
from contextlib import suppress
from pathlib import Path
from typing import Iterable, Optional, Sequence

from roarm_app.launch import extensions
from roarm_app.runtime.exit_codes import ExitCode

_USD_MODULE_CANDIDATE_ROOTS: Sequence[Path] = (
    Path(os.environ.get("ISAAC_SIM_ROOT", "")),
    Path(os.environ.get("ISAAC_SIM_PATH", "")),
    Path.home() / "isaacsim",
)

_USD_IMPORT_CACHE: dict[str, object] = {}

_KIT_SHUTDOWN_DONE: bool = False

def drain_updates(sim_app: object, iterations: int = 3) -> None:
    """Pump a SimulationApp a limited number of times to flush pending events."""

    for _ in range(max(iterations, 0)):
        if sim_app is None:
            return
        update = getattr(sim_app, "update", None)
        if not callable(update):
            return
        try:
            update()
        except Exception:
            return


def cancel_asyncio_tasks(prefix: str, *, timeout: float = 1.0) -> int:
    """Best-effort cancellation of pending asyncio tasks on the current loop."""

    loop: asyncio.AbstractEventLoop | None = None
    try:
        loop = asyncio.get_running_loop()
    except RuntimeError:
        pass

    if loop is None:
        try:
            loop = asyncio.get_event_loop_policy().get_event_loop()
        except RuntimeError:
            loop = None

    if loop is None or loop.is_closed():
        return 0

    pending = [task for task in asyncio.all_tasks(loop) if not task.done()]
    if not pending:
        return 0

    print(f"{prefix} asyncio pending tasks={len(pending)} — cancel 시작")
    for task in pending:
        task_name = getattr(task, "get_name", lambda: repr(task))()
        print(f"{prefix} asyncio cancel -> {task_name}")
        task.cancel()

    if loop.is_running():
        print(f"{prefix} asyncio 루프가 실행 중입니다. 다음 틱에서 취소가 처리되도록 요청했습니다.")
        return len(pending)

    try:
        loop.run_until_complete(
            asyncio.wait_for(asyncio.gather(*pending, return_exceptions=True), timeout)
        )
    except Exception as exc:  # pragma: no cover - 루프 종료 시 예외 무시
        print(f"{prefix} 경고: asyncio 대기 중 예외 발생 ({exc})")

    return len(pending)


def stop_timeline() -> None:
    """Stop the Kit timeline if it is currently playing."""

    try:
        timeline = importlib.import_module("omni.timeline")
    except ModuleNotFoundError:
        return

    get_timeline = getattr(timeline, "get_timeline_interface", None)
    if not callable(get_timeline):
        return

    with suppress(Exception):
        interface = get_timeline()
        if interface is not None and getattr(interface, "is_playing", lambda: False)():
            interface.stop()


def cleanup_usd_context(prefix: str, sim_app: object | None) -> object | None:
    """Close the active USD stage and detach event streams before shutdown."""

    try:
        ou_final = importlib.import_module("omni.usd")
    except ModuleNotFoundError:
        return None

    ctx_final = getattr(ou_final, "get_context", lambda: None)()
    if ctx_final is None:
        return None

    stage_copy: object | None = None

    try:
        event_stream_getter = getattr(ctx_final, "get_stage_event_stream", None)
        if callable(event_stream_getter):
            with suppress(Exception):
                event_stream = event_stream_getter()
                unsubscribe_all = getattr(event_stream, "unsubscribe_all", None)
                if callable(unsubscribe_all):
                    unsubscribe_all()

        stage_obj = ctx_final.get_stage()
        stage_copy = stage_obj
        if stage_obj is not None:
            with suppress(Exception):
                close_stage = getattr(ctx_final, "close_stage", None)
                if callable(close_stage):
                    close_stage()
            with suppress(Exception):
                stage_close = getattr(stage_obj, "Close", None)
                if callable(stage_close):
                    stage_close()
            with suppress(Exception):
                detach_stage = getattr(ctx_final, "detach_stage", None)
                if callable(detach_stage):
                    try:
                        detach_stage(stage_obj)
                    except TypeError:
                        detach_stage()
            with suppress(Exception):
                set_stage = getattr(ctx_final, "set_stage", None)
                if callable(set_stage):
                    set_stage(None)

        drain_updates(sim_app, 2)
    except Exception:
        return None

    return stage_copy


def release_physx_context(prefix: str, stage: object | None) -> None:
    """Attempt to tear down outstanding PhysX scenes to avoid loose references."""

    try:
        physx_utils = importlib.import_module("omni.physx.scripts.utils")
    except ModuleNotFoundError:
        return

    for func_name in ("destroy_stage", "destroy_stage_physics", "destroy_all_physx_scenes", "reset_physics"):
        func = getattr(physx_utils, func_name, None)
        if not callable(func):
            continue
        with suppress(Exception):
            try:
                if stage is not None:
                    func(stage)
                else:
                    func()
            except TypeError:
                func()


def ensure_usd_modules() -> tuple[object, object, object]:
    """Import pxr modules, extending sys.path if necessary for Isaac Sim installs."""

    cached = _USD_IMPORT_CACHE.get("modules")
    if cached is not None:
        return cached  # type: ignore[return-value]

    try:
        from pxr import Gf as _Gf, Usd as _Usd, UsdGeom as _UsdGeom  # type: ignore[import-not-found]

        _USD_IMPORT_CACHE["modules"] = (_Gf, _Usd, _UsdGeom)
        return _Gf, _Usd, _UsdGeom
    except ModuleNotFoundError:
        pass

    for root in _USD_MODULE_CANDIDATE_ROOTS:
        if not root:
            continue
        expanded = root.expanduser().resolve()
        if not expanded.exists():
            continue
        pxr_dirs = sorted(expanded.glob("extscache/omni.usd.libs-*/pxr"))
        if not pxr_dirs:
            continue
        pxr_root = pxr_dirs[-1]
        pxr_parent = pxr_root.parent
        bin_dir = pxr_parent / "bin"
        deps_lib_dir = pxr_parent / "bin/deps/lib"

        if str(pxr_parent) not in sys.path:
            sys.path.append(str(pxr_parent))

        current_py = os.environ.get("PYTHONPATH", "")
        py_parts = [p for p in current_py.split(":") if p]
        if str(pxr_parent) not in py_parts:
            py_parts.append(str(pxr_parent))
        os.environ["PYTHONPATH"] = ":".join(py_parts)

        ld_candidates = [bin_dir, deps_lib_dir]
        current_ld = os.environ.get("LD_LIBRARY_PATH", "")
        ld_parts = [p for p in current_ld.split(":") if p]
        for candidate in ld_candidates:
            if candidate.exists() and str(candidate) not in ld_parts:
                ld_parts.append(str(candidate))
        os.environ["LD_LIBRARY_PATH"] = ":".join(ld_parts)
        try:
            from pxr import Gf as _Gf, Usd as _Usd, UsdGeom as _UsdGeom  # type: ignore[import-not-found]
        except ModuleNotFoundError:
            continue
        _USD_IMPORT_CACHE["modules"] = (_Gf, _Usd, _UsdGeom)
        return _Gf, _Usd, _UsdGeom

    raise ModuleNotFoundError(
        "`pxr` 모듈을 찾을 수 없습니다. ISAAC_SIM_ROOT/ISAAC_SIM_PATH 환경변수를"
        " 재확인하고, omni.usd.libs-* 패키지가 설치되었는지 확인하세요."
    )


def sample_transform(prim: object) -> Optional[dict[str, list[float]]]:
    """Sample a prim's transform in world space for trajectory logging."""

    if prim is None:
        return None

    Gf, Usd, UsdGeom = ensure_usd_modules()

    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        return None

    matrix = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    translation = matrix.ExtractTranslation()
    rotation = matrix.ExtractRotation().GetQuat()
    return {
        "position": [float(translation[i]) for i in range(3)],
        "orientation_xyzw": [
            float(rotation.GetImaginary()[0]),
            float(rotation.GetImaginary()[1]),
            float(rotation.GetImaginary()[2]),
            float(rotation.GetReal()),
        ],
    }


def launch_simulation(
    *,
    args: Namespace,
    usd_path: Path,
    headless_mode: bool,
    blocklist_enabled: bool,
    applied_blocklist: Iterable[str],
    experience_override: Optional[str],
    run_notes: list[str],
    log_prefix: str = "open_roarm_m3_gui",
    debug_trace: bool = False,
) -> ExitCode:
    """Launch Isaac Sim, run the simulation loop, and return the exit reason."""

    prefix = f"[{log_prefix}]"

    exit_reason = "unresolved"

    exit_reason = ExitCode.UNRESOLVED

    if debug_trace:
        print(f"{prefix} [debug] 요청된 USD: {usd_path}")
        print(
            f"{prefix} [debug] 실행 모드: headless={headless_mode} blocklist={blocklist_enabled} overlay={experience_override!s}"
        )
        print(
            f"{prefix} [debug] OMNI_KIT_DISABLE_EXTENSIONS={os.environ.get('OMNI_KIT_DISABLE_EXTENSIONS', '')}"
        )

    try:
        SimulationApp = importlib.import_module("isaacsim").SimulationApp  # type: ignore[attr-defined]
    except ImportError as exc:  # pragma: no cover - 환경 특화
        raise SystemExit(
            "Isaac Sim Python 환경이 활성화되어 있지 않습니다. "
            "`source scripts/activate_isaacsim_env.sh` 등으로 환경을 준비한 뒤 다시 실행하세요."
        ) from exc

    sim_config: dict[str, object] = {"fast_shutdown": False}
    if headless_mode:
        sim_config.update(
            {
                "headless": True,
                "hide_ui": True,
                "disable_viewport_updates": True,
            }
        )
    else:
        sim_config.update({"headless": False, "hide_ui": False})

    if debug_trace:
        print(f"{prefix} [debug] SimulationApp config: {sim_config}")

    sim_app: object | None = None
    kit_app: object | None = None
    kit_manager: object | None = None
    try:
        if experience_override:
            sim_app = SimulationApp(sim_config, experience=experience_override)
        else:
            sim_app = SimulationApp(sim_config)
    except ModuleNotFoundError as exc:
        if experience_override and "omni.kit.usd" in str(exc):
            print(
                f"{prefix} 경고: headless overlay 적용 중 omni.kit.usd 모듈을 불러오지 못했습니다. 기본 experience로 재시도합니다."
            )
            sim_app = SimulationApp(sim_config)
            experience_override = None
        else:
            raise
    else:
        if experience_override:
            print(f"{prefix} headless overlay 적용: {experience_override}")

    if debug_trace:
        print(f"{prefix} [debug] SimulationApp 인스턴스 생성 완료")

    if headless_mode and blocklist_enabled and sim_app is not None:
        app_interface = getattr(sim_app, "get_app", None)
        if callable(app_interface):
            try:
                kit_app = app_interface()
                ext_mgr = getattr(kit_app, "get_extension_manager", None)
                if callable(ext_mgr):
                    kit_manager = ext_mgr()
                    blocklist = extensions.expand_prefix_blocklist(kit_manager, set(applied_blocklist))

                    extensions.sync_extension_blocklist(kit_manager, blocklist)
            except Exception:
                kit_app = None
                kit_manager = None

    try:
        ou = importlib.import_module("omni.usd")
    except ImportError as exc:  # pragma: no cover - 환경 특화
        raise RuntimeError("omni.usd 모듈을 불러올 수 없습니다. Isaac Sim 설치를 확인하세요.") from exc

    stage: object | None = None

    try:
        ctx = ou.get_context()
        ctx.open_stage(str(usd_path))
        stage = ctx.get_stage()
        poll_iterations = 0
        stage_poll_limit = 600
        while stage is None and poll_iterations < stage_poll_limit:
            sim_app.update()
            stage = ctx.get_stage()
            poll_iterations += 1

        if stage is None:
            run_notes.append("stage_poll_timeout")
            exit_reason = ExitCode.USD_STAGE_OPEN_FAILED
            raise RuntimeError("USD 스테이지 로딩 실패")

        run_notes.append(f"stage_poll={poll_iterations}")

        if debug_trace:
            print(f"{prefix} [debug] USD 스테이지 로드 완료 (polls={poll_iterations})")

        start_time = time.perf_counter()
        trajectory_records: list[dict[str, object]] = []
        trajectory_path = (
            args.trajectory_log.expanduser().resolve() if getattr(args, "trajectory_log", None) else None
        )
        usd_export_path = args.usd_export.expanduser().resolve() if getattr(args, "usd_export", None) else None

        def on_step(frame_index: int) -> None:
            if not trajectory_path:
                return
            prim_path = getattr(args, "prim", None)
            prim = stage.GetPrimAtPath(prim_path) if prim_path else None
            sample = sample_transform(prim)
            if sample is None:
                return
            trajectory_records.append(
                {
                    "frame": frame_index,
                    "elapsed_seconds": time.perf_counter() - start_time,
                    "prim_path": prim_path,
                    **sample,
                }
            )

        frame_counter = 0
        warmup_steps = max(getattr(args, "warmup_steps", 0), 0)
        warmup_executed = 0
        for _ in range(warmup_steps):
            if not sim_app.is_running():
                run_notes.append("warmup_aborted")
                break
            sim_app.update()
            on_step(frame_counter)
            frame_counter += 1
            warmup_executed += 1

        if warmup_steps:
            run_notes.append(f"warmup_steps={warmup_executed}")
            if debug_trace:
                print(f"{prefix} [debug] Warmup 실행 완료 (steps={warmup_executed})")

        if not headless_mode and getattr(args, "prim", None):
            try:
                viewport_util = importlib.import_module("omni.kit.viewport.utility")
                get_active_viewport_window = getattr(viewport_util, "get_active_viewport_window", None)
                focus_prim = getattr(viewport_util, "focus_prim", None)
                focus_prim_path = getattr(viewport_util, "focus_prim_path", None)
            except Exception:  # pragma: no cover - 선택 기능
                get_active_viewport_window = None  # type: ignore
                focus_prim = None
                focus_prim_path = None

            prim_focused = False
            prim_path = getattr(args, "prim", None)
            if callable(focus_prim_path) and prim_path:
                try:
                    focus_prim_path(prim_path)
                    prim_focused = True
                except Exception as exc:  # pragma: no cover - 포커스 실패 시 기록
                    print(f"{prefix} 경고: focus_prim_path 호출 실패 ({exc})")
            elif callable(focus_prim) and prim_path:
                try:
                    focus_prim(prim_path)
                    prim_focused = True
                except Exception as exc:  # pragma: no cover - 포커스 실패 시 기록
                    print(f"{prefix} 경고: focus_prim 호출 실패 ({exc})")

            if not prim_focused and callable(get_active_viewport_window):
                viewport = get_active_viewport_window()
                focus_method = getattr(viewport, "focus_prim_path", None) if viewport else None
                if callable(focus_method) and prim_path:
                    try:
                        focus_method(prim_path)
                        prim_focused = True
                    except Exception as exc:  # pragma: no cover - 포커스 실패 시 기록
                        print(f"{prefix} 경고: 뷰포트 focus_prim_path 호출 실패 ({exc})")

            if not prim_focused:
                print(f"{prefix} 정보: prim 포커스 생략 (사용 가능한 포커스 API 미탐지) prim={prim_path}")

        if headless_mode:
            print(f"{prefix} Headless 학습 모드로 시뮬레이션을 유지합니다.")
            train_steps = 0
            max_train_steps = getattr(args, "max_steps", 0) if getattr(args, "max_steps", 0) > 0 else None
            while sim_app.is_running():
                sim_app.update()
                on_step(frame_counter)
                frame_counter += 1
                train_steps += 1
                if max_train_steps is not None and train_steps >= max_train_steps:
                    break
                if train_steps % 2400 == 0:
                    elapsed = time.perf_counter() - start_time
                    print(f"{prefix} headless loop steps={train_steps}, elapsed={elapsed:.1f}s")

            if max_train_steps is not None and train_steps >= max_train_steps:
                exit_reason = ExitCode.MAX_STEPS_REACHED
            else:
                exit_reason = ExitCode.IS_RUNNING_FALSE

            run_notes.append(f"train_steps={train_steps}")
            print(f"{prefix} Headless loop 종료 (steps={train_steps})")
            if debug_trace:
                print(
                    f"{prefix} [debug] Headless 루프 종료 사유: exit_reason={exit_reason.value} is_running={sim_app.is_running()}"
                )
        else:
            print(f"{prefix} Isaac Sim GUI loop 시작")
            frames = frame_counter
            max_ui_frames = getattr(args, "max_steps", 0) if getattr(args, "max_steps", 0) > 0 else None
            while sim_app.is_running():
                sim_app.update()
                on_step(frames)
                frames += 1
                if max_ui_frames is not None and frames >= max_ui_frames:
                    break
                if frames % 60 == 0:
                    print(f"{prefix} UI loop frames={frames}")

            if max_ui_frames is not None and frames >= max_ui_frames:
                exit_reason = ExitCode.MAX_STEPS_REACHED
            else:
                exit_reason = ExitCode.IS_RUNNING_FALSE

            run_notes.append(f"ui_frames={frames}")
            print(f"{prefix} Isaac Sim UI loop 종료 (frames={frames})")
            if debug_trace:
                print(
                    f"{prefix} [debug] GUI 루프 종료 사유: exit_reason={exit_reason.value} is_running={sim_app.is_running()}"
                )

        if trajectory_path:
            trajectory_path.parent.mkdir(parents=True, exist_ok=True)
            with trajectory_path.open("w", encoding="utf-8") as fp:
                json.dump(
                    {
                        "usd": str(usd_path),
                        "mode": getattr(args, "mode", "train"),
                        "prim": getattr(args, "prim", None),
                        "frames": trajectory_records,
                    },
                    fp,
                    ensure_ascii=False,
                    indent=2,
                )
            print(f"{prefix} 트래젝토리 로그 저장 완료: {trajectory_path} (frames={len(trajectory_records)})")

        if usd_export_path:
            usd_export_path.parent.mkdir(parents=True, exist_ok=True)
            stage.Export(str(usd_export_path))
            print(f"{prefix} 스테이지를 {usd_export_path} 로 Export 하였습니다.")
    finally:
        stage = None
        exit_reason = shutdown(
            sim_app=sim_app,
            kit_app=kit_app,
            log_prefix=log_prefix,
            run_notes=run_notes,
            exit_reason=exit_reason,
        )
        if debug_trace:
            print(f"{prefix} [debug] shutdown() 완료 exit_reason={exit_reason.value}")

    return exit_reason


def shutdown(
    *,
    sim_app: object | None,
    kit_app: object | None,
    log_prefix: str,
    run_notes: list[str],
    exit_reason: ExitCode,
) -> ExitCode:
    """Handle Kit shutdown and emit final run summary."""

    global _KIT_SHUTDOWN_DONE

    prefix = f"[{log_prefix}]"

    stop_timeline()

    if sim_app is not None:
        stage_ref = cleanup_usd_context(prefix, sim_app)
        release_physx_context(prefix, stage_ref)
        gc.collect()
        drain_updates(sim_app, 1)
        pending_asyncio = cancel_asyncio_tasks(prefix, timeout=2.0)
        if pending_asyncio:
            run_notes.append(f"asyncio_cancelled={pending_asyncio}")

        try:
            if kit_app is None:
                kit_app = getattr(sim_app, "app", None)
                if kit_app is None:
                    app_getter = getattr(sim_app, "get_app", None)
                    if callable(app_getter):
                        kit_app = app_getter()
            shutdown_fn = getattr(kit_app, "shutdown", None) if kit_app is not None else None
            if callable(shutdown_fn):
                if _KIT_SHUTDOWN_DONE:
                    run_notes.append("kit_shutdown=skipped")
                else:
                    try:
                        shutdown_fn()
                    except TypeError:
                        shutdown_fn(False)
                    _KIT_SHUTDOWN_DONE = True
        except Exception:
            pass

        drain_updates(sim_app, 2)
        with suppress(Exception):
            sim_app.close()

    if exit_reason is ExitCode.UNRESOLVED:
        exit_reason = ExitCode.COMPLETED
    if run_notes:
        print(f"{prefix} RUN_NOTES={';'.join(run_notes)}")
    print(f"{prefix} EXIT_REASON={exit_reason.value}")
    return exit_reason
