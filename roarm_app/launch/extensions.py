"""Extension management helpers for Isaac Sim headless launch."""

from __future__ import annotations

from typing import Iterable, Sequence, Set

_GUI_ONLY_EXTENSIONS: Sequence[str] = (
    "isaacsim.app.about",
    "isaacsim.gui.components",
    "isaacsim.gui.content_browser",
    "isaacsim.gui.menu",
    "isaacsim.gui.property",
    "isaacsim.gui.sensors.icon",
    "omni.kit.ui.editor_menu",
    "omni.kit.menu.common",
    "omni.kit.menu.core",
    "omni.kit.menu.create",
    "omni.kit.menu.stage",
    "omni.kit.menu.utils",
    "omni.kit.renderer.capture",
    "omni.kit.renderer.imgui",
    "omni.kit.uiapp",
    "omni.kit.viewport.actions",
    "omni.kit.viewport.manipulator.camera",
    "omni.kit.viewport.manipulator.transform",
    "omni.kit.viewport.menubar.camera",
    "omni.kit.viewport.menubar.core",
    "omni.kit.viewport.menubar.display",
    "omni.kit.viewport.menubar.lighting",
    "omni.kit.viewport.menubar.render",
    "omni.kit.viewport.menubar.settings",
    "omni.kit.viewport.registry",
    "omni.kit.viewport.utility",
    "omni.kit.viewport.window",
    "omni.kit.widget.browser_bar",
    "omni.kit.widget.context_menu",
    "omni.kit.widget.filebrowser",
    "omni.kit.widget.graph",
    "omni.kit.widget.highlight_label",
    "omni.kit.widget.live",
    "omni.kit.widget.options_button",
    "omni.kit.widget.options_menu",
    "omni.kit.widget.path_field",
    "omni.kit.widget.prompt",
    "omni.kit.widget.searchable_combobox",
    "omni.kit.widget.searchfield",
    "omni.kit.widget.stage",
    "omni.kit.widget.stage_icons",
    "omni.kit.widget.toolbar",
    "omni.kit.widget.viewport",
    "omni.kit.widget.zoombar",
    "omni.kit.window.about",
    "omni.kit.window.console",
    "omni.kit.window.content",
    "omni.kit.window.content_browser",
    "omni.kit.window.content_browser_registry",
    "omni.kit.window.extensions",
    "omni.kit.window.file",
    "omni.kit.window.file_exporter",
    "omni.kit.window.file_importer",
    "omni.kit.window.filepicker",
    "omni.kit.window.material_graph",
    "omni.kit.window.popup_dialog",
    "omni.kit.window.preferences",
    "omni.kit.window.property",
    "omni.kit.window.script_editor",
    "omni.kit.window.stage",
    "omni.kit.window.status_bar",
    "omni.kit.window.title",
    "omni.kit.window.toolbar",
    "omni.kit.window.viewport",
    "omni.kit.manipulator",
    "omni.kit.manipulator.camera",
    "omni.kit.manipulator.prim",
    "omni.kit.manipulator.prim.core",
    "omni.kit.manipulator.prim.usd",
    "omni.kit.manipulator.prim.fabric",
    "omni.kit.manipulator.selection",
    "omni.kit.manipulator.selector",
    "omni.kit.manipulator.tool.snap",
    "omni.kit.manipulator.transform",
    "omni.kit.manipulator.viewport",
    "isaacsim.robot.manipulators",
    "omni.isaac.manipulators",
)

_REPLICATOR_EXTENSIONS: Sequence[str] = (
    "omni.replicator.core",
    "omni.replicator.isaac",
    "omni.replicator.isaac_ext",
    "omni.replicator.replicator_yaml",
    "omni.syntheticdata",
    "omni.syntheticdata.asynchronous",
    "omni.syntheticdata.ui",
    "isaacsim.replicator.behavior",
    "isaacsim.replicator.domain_randomization",
    "isaacsim.replicator.examples",
    "isaacsim.replicator.writers",
)

_AUDIO_EXTENSIONS: Sequence[str] = (
    "carb.audio",
    "omni.uiaudio",
    "omni.kit.audiodeviceenum",
    "omni.kit.property.audio",
)

_BLOCKLIST_PREFIXES: Sequence[str] = (
    "isaacsim.gui",
    "isaacsim.robot.manipulators",
    "isaacsim.replicator",
    "omni.isaac.manipulators",
    "omni.graph.replicator",
    "omni.kit.menu",
    "omni.kit.viewport",
    "omni.kit.widget",
    "omni.kit.window",
    "omni.kit.manipulator",
    "omni.replicator",
    "omni.services.replicator",
    "omni.syntheticdata",
    "carb.audio",
    "omni.uiaudio",
    "omni.kit.property.audio",
)


def collect_extension_blocklist(replicator_should_disable: bool) -> Set[str]:
    """Derive the baseline extension blocklist for headless mode."""

    blocklist = set(_GUI_ONLY_EXTENSIONS)
    if replicator_should_disable:
        blocklist.update(_REPLICATOR_EXTENSIONS)
    blocklist.update(_AUDIO_EXTENSIONS)
    return blocklist


def sync_extension_blocklist(manager: object, blocklist: Iterable[str]) -> None:
    """Deactivate extensions through the Kit extension manager when possible."""

    if manager is None:
        return

    is_enabled = getattr(manager, "is_extension_enabled", None)
    set_enabled = getattr(manager, "set_extension_enabled", None)
    update_states = getattr(manager, "update_extension_states", None)
    if not callable(set_enabled) or not callable(update_states):
        return

    for ext_name in blocklist:
        try:
            if not callable(is_enabled) or is_enabled(ext_name):
                set_enabled(ext_name, False)
        except Exception:
            continue

    try:
        update_states()
    except Exception:
        pass


def expand_prefix_blocklist(manager: object, baseline: Set[str]) -> Set[str]:
    """Augment a blocklist with any already-enabled extensions matching known prefixes."""

    blocklist = set(baseline)
    if manager is None:
        return blocklist

    getter = getattr(manager, "get_enabled_extension_module_names", None)
    if not callable(getter):
        return blocklist

    try:
        enabled_modules = tuple(getter())
    except Exception:
        return blocklist

    for module_name in enabled_modules:
        if any(module_name.startswith(prefix) for prefix in _BLOCKLIST_PREFIXES):
            blocklist.add(module_name)

    return blocklist
