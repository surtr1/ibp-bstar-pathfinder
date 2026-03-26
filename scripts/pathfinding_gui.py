from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

try:
    from PySide6 import QtCore, QtGui, QtWidgets
except ModuleNotFoundError as exc:
    raise SystemExit("PySide6 is not installed. Run: pip install -r scripts/requirements-gui.txt") from exc


class CompareWorker(QtCore.QObject):
    finished = QtCore.Signal(dict, str)
    failed = QtCore.Signal(str, str)
    done = QtCore.Signal()

    def __init__(self, command: list[str], working_directory: Path) -> None:
        super().__init__()
        self._command = command
        self._working_directory = working_directory

    @QtCore.Slot()
    def run(self) -> None:
        try:
            completed = subprocess.run(
                self._command,
                cwd=str(self._working_directory),
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
            )

            stdout_text = completed.stdout.strip()
            stderr_text = completed.stderr.strip()
            parsed_payload: dict | None = None

            if stdout_text:
                try:
                    parsed_payload = json.loads(stdout_text)
                except json.JSONDecodeError as error:
                    self.failed.emit(f"Failed to parse JSON output: {error}", stdout_text or stderr_text)
                    return

            if completed.returncode != 0:
                if isinstance(parsed_payload, dict) and "error" in parsed_payload:
                    self.failed.emit(str(parsed_payload["error"]), stdout_text or stderr_text)
                else:
                    self.failed.emit(stderr_text or stdout_text or "The executable returned a non-zero exit code.", stdout_text or stderr_text)
                return

            if not isinstance(parsed_payload, dict):
                self.failed.emit("The executable returned empty JSON output.", stdout_text or stderr_text)
                return

            if not parsed_payload.get("ok", True):
                self.failed.emit(str(parsed_payload.get("error", "Unknown error")), stdout_text or stderr_text)
                return

            self.finished.emit(parsed_payload, stdout_text)
        except FileNotFoundError:
            self.failed.emit("The selected executable was not found.", "")
        except Exception as error:  # pragma: no cover - GUI fallback path
            self.failed.emit(str(error), "")
        finally:
            self.done.emit()


class GridCanvas(QtWidgets.QWidget):
    cellClicked = QtCore.Signal(int, int)

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self._payload: dict | None = None
        self._selected_algorithm_name = ""
        self._overlay_all_paths = False
        self._base_cell_size = 18.0
        self._zoom_factor = 1.0
        self._padding = 24.0
        self._path_colors = {
            "BFS": QtGui.QColor("#5f6368"),
            "A*": QtGui.QColor("#1a73e8"),
            "Dijkstra": QtGui.QColor("#8e24aa"),
            "B* PaperCrawl": QtGui.QColor("#f57c00"),
            "B* GreedyLite": QtGui.QColor("#ffb74d"),
            "B* Robust": QtGui.QColor("#8d6e63"),
            "IBP-B*": QtGui.QColor("#2e7d32"),
        }
        self._wall_color = QtGui.QColor("#1f2933")
        self._empty_color = QtGui.QColor("#f8fafc")
        self._grid_line_color = QtGui.QColor("#d9e2ec")
        self._start_color = QtGui.QColor("#00a86b")
        self._goal_color = QtGui.QColor("#d62828")
        self.setAutoFillBackground(True)

    def sizeHint(self) -> QtCore.QSize:
        if not self._payload:
            return QtCore.QSize(720, 520)

        grid = self._payload["map"]["grid"]
        if not grid:
            return QtCore.QSize(720, 520)

        cell_size = self._current_cell_size()
        width = int(math.ceil(self._padding * 2 + len(grid[0]) * cell_size))
        height = int(math.ceil(self._padding * 2 + len(grid) * cell_size))
        return QtCore.QSize(max(width, 320), max(height, 240))

    def set_payload(self, payload: dict | None) -> None:
        self._payload = payload
        self._selected_algorithm_name = ""
        self._zoom_factor = 1.0

        if payload and payload.get("map"):
            grid = payload["map"]["grid"]
            max_dimension = max(len(grid), len(grid[0]) if grid else 1, 1)
            self._base_cell_size = max(1.0, min(24.0, 720.0 / max_dimension))
        else:
            self._base_cell_size = 18.0

        self.updateGeometry()
        self.adjustSize()
        self.update()

    def set_selected_algorithm_name(self, algorithm_name: str) -> None:
        self._selected_algorithm_name = algorithm_name
        self.update()

    def set_overlay_all_paths(self, enabled: bool) -> None:
        self._overlay_all_paths = enabled
        self.update()

    def zoom_in(self) -> None:
        self._zoom_factor = min(8.0, self._zoom_factor * 1.25)
        self.updateGeometry()
        self.adjustSize()
        self.update()

    def zoom_out(self) -> None:
        self._zoom_factor = max(0.25, self._zoom_factor / 1.25)
        self.updateGeometry()
        self.adjustSize()
        self.update()

    def reset_zoom(self) -> None:
        self._zoom_factor = 1.0
        self.updateGeometry()
        self.adjustSize()
        self.update()

    def _current_cell_size(self) -> float:
        return max(1.0, self._base_cell_size * self._zoom_factor)

    def _selected_algorithm(self) -> dict | None:
        if not self._payload:
            return None

        algorithms = self._payload.get("algorithms", [])
        if not algorithms:
            return None

        if not self._selected_algorithm_name:
            return algorithms[0]

        for algorithm in algorithms:
            if algorithm.get("name") == self._selected_algorithm_name:
                return algorithm
        return algorithms[0]

    def _successful_algorithms(self) -> list[dict]:
        if not self._payload:
            return []
        return [algorithm for algorithm in self._payload.get("algorithms", []) if algorithm.get("success") and algorithm.get("path")]

    def _draw_algorithm_path(self, painter: QtGui.QPainter, algorithm: dict, cell_size: float, highlighted: bool) -> None:
        path_color = QtGui.QColor(self._path_colors.get(algorithm.get("name", ""), QtGui.QColor("#1565c0")))
        if highlighted:
            line_width = max(2.0, cell_size * 0.45)
        else:
            path_color.setAlpha(150)
            line_width = max(1.5, cell_size * 0.28)

        path = algorithm["path"]
        polygon = QtGui.QPolygonF(
            [
                QtCore.QPointF(
                    self._padding + point["col"] * cell_size + cell_size / 2.0,
                    self._padding + point["row"] * cell_size + cell_size / 2.0,
                )
                for point in path
            ]
        )
        painter.setPen(QtGui.QPen(path_color, line_width, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap, QtCore.Qt.RoundJoin))
        painter.drawPolyline(polygon)

    def _draw_path_overlay(self, painter: QtGui.QPainter, cell_size: float) -> None:
        selected_algorithm = self._selected_algorithm()
        successful_algorithms = self._successful_algorithms()
        if not successful_algorithms:
            return

        if self._overlay_all_paths:
            for algorithm in successful_algorithms:
                if selected_algorithm and algorithm.get("name") == selected_algorithm.get("name"):
                    continue
                self._draw_algorithm_path(painter, algorithm, cell_size, highlighted=False)

            if selected_algorithm and selected_algorithm.get("success") and selected_algorithm.get("path"):
                self._draw_algorithm_path(painter, selected_algorithm, cell_size, highlighted=True)
            return

        if selected_algorithm and selected_algorithm.get("success") and selected_algorithm.get("path"):
            self._draw_algorithm_path(painter, selected_algorithm, cell_size, highlighted=True)

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QtGui.QColor("#f1f5f9"))

        if not self._payload or not self._payload.get("map"):
            painter.setPen(QtGui.QColor("#52606d"))
            painter.drawText(self.rect().adjusted(16, 16, -16, -16), QtCore.Qt.AlignCenter, "Run an algorithm comparison to preview the map and path.")
            return

        grid = self._payload["map"]["grid"]
        if not grid:
            painter.setPen(QtGui.QColor("#52606d"))
            painter.drawText(self.rect().adjusted(16, 16, -16, -16), QtCore.Qt.AlignCenter, "The map grid is empty.")
            return

        cell_size = self._current_cell_size()
        padding = self._padding
        clip_rect = event.rect()

        left_col = max(0, int(math.floor((clip_rect.left() - padding) / cell_size)))
        right_col = min(len(grid[0]) - 1, int(math.ceil((clip_rect.right() - padding) / cell_size)))
        top_row = max(0, int(math.floor((clip_rect.top() - padding) / cell_size)))
        bottom_row = min(len(grid) - 1, int(math.ceil((clip_rect.bottom() - padding) / cell_size)))

        for row in range(top_row, bottom_row + 1):
            for col in range(left_col, right_col + 1):
                rect = QtCore.QRectF(padding + col * cell_size, padding + row * cell_size, cell_size, cell_size)
                painter.fillRect(rect, self._wall_color if grid[row][col] == 1 else self._empty_color)
                if cell_size >= 8.0:
                    painter.setPen(self._grid_line_color)
                    painter.drawRect(rect)

        self._draw_path_overlay(painter, cell_size)

        start = self._payload["map"]["start"]
        goal = self._payload["map"]["goal"]
        self._draw_endpoint(painter, start, "S", self._start_color, cell_size)
        self._draw_endpoint(painter, goal, "E", self._goal_color, cell_size)

    def _draw_endpoint(self, painter: QtGui.QPainter, point: dict, label: str, fill_color: QtGui.QColor, cell_size: float) -> None:
        center_x = self._padding + point["col"] * cell_size + cell_size / 2.0
        center_y = self._padding + point["row"] * cell_size + cell_size / 2.0
        radius = max(4.0, cell_size * 0.45)

        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(fill_color)
        painter.drawEllipse(QtCore.QPointF(center_x, center_y), radius, radius)

        if cell_size >= 10.0:
            font = painter.font()
            font.setBold(True)
            font.setPointSizeF(max(7.0, min(18.0, cell_size * 0.55)))
            painter.setFont(font)
            painter.setPen(QtGui.QColor("#ffffff"))
            text_rect = QtCore.QRectF(center_x - radius, center_y - radius, radius * 2.0, radius * 2.0)
            painter.drawText(text_rect, QtCore.Qt.AlignCenter, label)

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() != QtCore.Qt.LeftButton or not self._payload or not self._payload.get("map"):
            super().mousePressEvent(event)
            return

        grid = self._payload["map"]["grid"]
        if not grid:
            super().mousePressEvent(event)
            return

        cell_size = self._current_cell_size()
        col = int((event.position().x() - self._padding) // cell_size)
        row = int((event.position().y() - self._padding) // cell_size)
        if 0 <= row < len(grid) and 0 <= col < len(grid[0]):
            self.cellClicked.emit(row, col)
            event.accept()
            return

        super().mousePressEvent(event)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self._workspace_root = Path(__file__).resolve().parents[1]
        self._worker_thread: QtCore.QThread | None = None
        self._worker: CompareWorker | None = None
        self._payload: dict | None = None
        self._pending_pick_target: str | None = None

        self.setWindowTitle("Pathfinding Compare Viewer")
        self.resize(1480, 900)

        self._build_ui()
        self._apply_defaults()

    def _build_ui(self) -> None:
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)

        root_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        root_splitter.addWidget(self._build_controls_panel())
        root_splitter.addWidget(self._build_view_panel())
        root_splitter.setChildrenCollapsible(False)
        root_splitter.setStretchFactor(0, 0)
        root_splitter.setStretchFactor(1, 1)
        root_splitter.setSizes([460, 1020])

        layout = QtWidgets.QHBoxLayout(central_widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.addWidget(root_splitter)

        self.statusBar().showMessage("Ready")

    def _build_controls_panel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QScrollArea()
        panel.setWidgetResizable(True)
        panel.setFrameShape(QtWidgets.QFrame.NoFrame)
        panel.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        panel.setMinimumWidth(460)
        panel.setSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.Preferred)

        content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(content)
        layout.setContentsMargins(0, 0, 8, 0)
        layout.setSpacing(10)

        layout.addWidget(self._build_executable_group())
        layout.addWidget(self._build_map_group())
        layout.addWidget(self._build_endpoint_group())
        layout.addWidget(self._build_algorithm_group())
        layout.addWidget(self._build_options_group())
        layout.addStretch(1)

        self.run_button = QtWidgets.QPushButton("Run Comparison")
        self.run_button.clicked.connect(self._run_comparison)
        layout.addWidget(self.run_button)

        panel.setWidget(content)
        return panel

    def _build_view_panel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)

        toolbar_layout = QtWidgets.QHBoxLayout()
        toolbar_layout.addWidget(QtWidgets.QLabel("Focus Path:"))
        self.algorithm_combo = QtWidgets.QComboBox()
        self.algorithm_combo.currentTextChanged.connect(self._on_algorithm_changed)
        toolbar_layout.addWidget(self.algorithm_combo, 1)

        self.overlay_paths_checkbox = QtWidgets.QCheckBox("Overlay all paths")
        self.overlay_paths_checkbox.toggled.connect(self._on_overlay_toggled)
        toolbar_layout.addWidget(self.overlay_paths_checkbox)

        zoom_out_button = QtWidgets.QPushButton("Zoom -")
        zoom_out_button.clicked.connect(self._zoom_out)
        toolbar_layout.addWidget(zoom_out_button)

        zoom_in_button = QtWidgets.QPushButton("Zoom +")
        zoom_in_button.clicked.connect(self._zoom_in)
        toolbar_layout.addWidget(zoom_in_button)

        reset_zoom_button = QtWidgets.QPushButton("Reset Zoom")
        reset_zoom_button.clicked.connect(self._reset_zoom)
        toolbar_layout.addWidget(reset_zoom_button)

        layout.addLayout(toolbar_layout)

        self.overlay_hint_label = QtWidgets.QLabel("Tip: when overlay is enabled, the focus path is drawn on top with a thicker line.")
        self.overlay_hint_label.setStyleSheet("color: #52606d;")
        layout.addWidget(self.overlay_hint_label)

        view_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)

        self.canvas = GridCanvas()
        self.canvas.cellClicked.connect(self._on_canvas_cell_clicked)
        self.canvas_scroll_area = QtWidgets.QScrollArea()
        self.canvas_scroll_area.setWidgetResizable(False)
        self.canvas_scroll_area.setBackgroundRole(QtGui.QPalette.Base)
        self.canvas_scroll_area.setWidget(self.canvas)
        view_splitter.addWidget(self.canvas_scroll_area)

        bottom_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        self.results_table = QtWidgets.QTableWidget(0, 6)
        self.results_table.setHorizontalHeaderLabels(["Algorithm", "Success", "PathLen", "Expanded", "Time(us)", "GapToBFS"])
        self.results_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.results_table.verticalHeader().setVisible(False)
        self.results_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.results_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.results_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.results_table.itemSelectionChanged.connect(self._on_table_selection_changed)
        bottom_splitter.addWidget(self.results_table)

        self.output_text = QtWidgets.QPlainTextEdit()
        self.output_text.setReadOnly(True)
        bottom_splitter.addWidget(self.output_text)
        bottom_splitter.setStretchFactor(0, 1)
        bottom_splitter.setStretchFactor(1, 1)

        view_splitter.addWidget(bottom_splitter)
        view_splitter.setStretchFactor(0, 3)
        view_splitter.setStretchFactor(1, 2)

        layout.addWidget(view_splitter, 1)
        return panel

    def _build_executable_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Executable")
        layout = QtWidgets.QGridLayout(group)

        self.exe_path_edit = QtWidgets.QLineEdit()
        browse_button = QtWidgets.QPushButton("Browse...")
        browse_button.clicked.connect(self._browse_executable)

        layout.addWidget(QtWidgets.QLabel("pathfinding_compare:"), 0, 0)
        layout.addWidget(self.exe_path_edit, 1, 0)
        layout.addWidget(browse_button, 1, 1)
        return group

    def _build_map_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Map Source")
        layout = QtWidgets.QGridLayout(group)

        self.map_mode_combo = QtWidgets.QComboBox()
        self.map_mode_combo.addItems(["File Map", "Random Map", "Staggered Walls", "Perfect Maze"])
        self.map_mode_combo.currentIndexChanged.connect(self._update_map_controls)

        self.map_file_edit = QtWidgets.QLineEdit()
        self.map_browse_button = QtWidgets.QPushButton("Browse...")
        self.map_browse_button.clicked.connect(self._browse_map_file)

        self.random_width_spin = QtWidgets.QSpinBox()
        self.random_width_spin.setRange(3, 5000)
        self.random_width_spin.setValue(64)
        self.random_width_spin.valueChanged.connect(self._update_endpoint_ranges)

        self.random_height_spin = QtWidgets.QSpinBox()
        self.random_height_spin.setRange(3, 5000)
        self.random_height_spin.setValue(64)
        self.random_height_spin.valueChanged.connect(self._update_endpoint_ranges)

        self.wall_probability_spin = QtWidgets.QDoubleSpinBox()
        self.wall_probability_spin.setRange(0.0, 1.0)
        self.wall_probability_spin.setSingleStep(0.01)
        self.wall_probability_spin.setDecimals(3)
        self.wall_probability_spin.setValue(0.25)

        self.seed_spin = QtWidgets.QSpinBox()
        self.seed_spin.setRange(0, 2_147_483_647)
        self.seed_spin.setValue(12345)

        layout.addWidget(QtWidgets.QLabel("Mode:"), 0, 0)
        layout.addWidget(self.map_mode_combo, 0, 1, 1, 2)
        layout.addWidget(QtWidgets.QLabel("Map file:"), 1, 0)
        layout.addWidget(self.map_file_edit, 1, 1)
        layout.addWidget(self.map_browse_button, 1, 2)
        layout.addWidget(QtWidgets.QLabel("Width:"), 2, 0)
        layout.addWidget(self.random_width_spin, 2, 1)
        layout.addWidget(QtWidgets.QLabel("Height:"), 3, 0)
        layout.addWidget(self.random_height_spin, 3, 1)
        self.wall_parameter_label = QtWidgets.QLabel("Wall p:")
        layout.addWidget(self.wall_parameter_label, 4, 0)
        layout.addWidget(self.wall_probability_spin, 4, 1)
        layout.addWidget(QtWidgets.QLabel("Seed:"), 5, 0)
        layout.addWidget(self.seed_spin, 5, 1)
        return group

    def _build_endpoint_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Start / Goal")
        layout = QtWidgets.QGridLayout(group)
        layout.setHorizontalSpacing(12)
        layout.setColumnStretch(1, 1)

        self.start_override_checkbox = QtWidgets.QCheckBox("Override Start")
        self.goal_override_checkbox = QtWidgets.QCheckBox("Override Goal")

        self.start_row_spin = QtWidgets.QSpinBox()
        self.start_row_spin.setRange(0, 9999)
        self.start_col_spin = QtWidgets.QSpinBox()
        self.start_col_spin.setRange(0, 9999)

        self.goal_row_spin = QtWidgets.QSpinBox()
        self.goal_row_spin.setRange(0, 9999)
        self.goal_col_spin = QtWidgets.QSpinBox()
        self.goal_col_spin.setRange(0, 9999)

        self.pick_start_button = QtWidgets.QPushButton("Pick Start")
        self.pick_start_button.setCheckable(True)
        self.pick_start_button.toggled.connect(lambda checked: self._set_pick_mode("start", checked))
        self.pick_start_button.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.pick_start_button.setMinimumHeight(30)
        self.pick_start_button.setMinimumWidth(170)

        self.pick_goal_button = QtWidgets.QPushButton("Pick Goal")
        self.pick_goal_button.setCheckable(True)
        self.pick_goal_button.toggled.connect(lambda checked: self._set_pick_mode("goal", checked))
        self.pick_goal_button.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.pick_goal_button.setMinimumHeight(30)
        self.pick_goal_button.setMinimumWidth(170)

        endpoint_hint = QtWidgets.QLabel("Tip: run once, then click the map to place a custom start or goal.")
        endpoint_hint.setWordWrap(True)
        endpoint_hint.setStyleSheet("color: #52606d;")

        layout.addWidget(self.start_override_checkbox, 0, 0, 1, 2)
        layout.addWidget(QtWidgets.QLabel("Start row:"), 1, 0)
        layout.addWidget(self.start_row_spin, 1, 1)
        layout.addWidget(QtWidgets.QLabel("Start col:"), 2, 0)
        layout.addWidget(self.start_col_spin, 2, 1)
        layout.addWidget(self.pick_start_button, 3, 0, 1, 2)

        layout.addWidget(self.goal_override_checkbox, 4, 0, 1, 2)
        layout.addWidget(QtWidgets.QLabel("Goal row:"), 5, 0)
        layout.addWidget(self.goal_row_spin, 5, 1)
        layout.addWidget(QtWidgets.QLabel("Goal col:"), 6, 0)
        layout.addWidget(self.goal_col_spin, 6, 1)
        layout.addWidget(self.pick_goal_button, 7, 0, 1, 2)
        layout.addWidget(endpoint_hint, 8, 0, 1, 2)
        return group

    def _build_algorithm_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Algorithms")
        layout = QtWidgets.QGridLayout(group)
        layout.setHorizontalSpacing(18)
        layout.setVerticalSpacing(8)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)

        self.algorithm_checkboxes: dict[str, QtWidgets.QCheckBox] = {}
        labels = ["BFS", "A*", "Dijkstra", "B* PaperCrawl", "B* GreedyLite", "B* Robust", "IBP-B*"]
        column_count = 2

        for index, label in enumerate(labels):
            checkbox = QtWidgets.QCheckBox(label)
            checkbox.setChecked(label not in {"B* GreedyLite", "B* Robust"})
            checkbox.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            self.algorithm_checkboxes[label] = checkbox

            row = index // column_count
            col = index % column_count
            layout.addWidget(checkbox, row, col)
        return group

    def _build_options_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Algorithm Options")
        layout = QtWidgets.QFormLayout(group)

        self.branch_no_reverse_checkbox = QtWidgets.QCheckBox("Disable B* reverse crawl")
        self.ibp_paper_strict_checkbox = QtWidgets.QCheckBox("Use IBP-B* paper-strict mode")
        self.ibp_wait_spin = QtWidgets.QSpinBox()
        self.ibp_wait_spin.setRange(0, 100)
        self.ibp_wait_spin.setValue(2)
        self.ibp_zigzag_threshold_spin = QtWidgets.QSpinBox()
        self.ibp_zigzag_threshold_spin.setRange(0, 100)
        self.ibp_zigzag_threshold_spin.setValue(2)

        layout.addRow(self.branch_no_reverse_checkbox)
        layout.addRow(self.ibp_paper_strict_checkbox)
        layout.addRow("IBP wait layers:", self.ibp_wait_spin)
        layout.addRow("IBP zigzag threshold:", self.ibp_zigzag_threshold_spin)
        return group

    def _apply_defaults(self) -> None:
        self.exe_path_edit.setText(str(self._default_executable_path()))
        self.map_file_edit.setText(str(self._workspace_root / "maps" / "ring_obstacle.txt"))
        self._update_map_controls()
        self._update_endpoint_ranges()

    def _default_executable_path(self) -> Path:
        candidates = [
            self._workspace_root / "pathfinding_compare.exe",
            self._workspace_root / "build" / "Release" / "pathfinding_compare.exe",
            self._workspace_root / "build" / "pathfinding_compare.exe",
            self._workspace_root / "output" / "pathfinding_compare_reorg.exe",
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate
        return self._workspace_root / "pathfinding_compare.exe"

    def _browse_executable(self) -> None:
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Select pathfinding_compare executable", str(self._workspace_root), "Executable (*.exe);;All Files (*)")
        if file_path:
            self.exe_path_edit.setText(file_path)

    def _browse_map_file(self) -> None:
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Select map file", str(self._workspace_root / "maps"), "Text Files (*.txt);;All Files (*)")
        if file_path:
            self.map_file_edit.setText(file_path)

    def _update_map_controls(self) -> None:
        is_file_mode = self.map_mode_combo.currentText() == "File Map"
        is_random_mode = self.map_mode_combo.currentText() == "Random Map"
        is_staggered_mode = self.map_mode_combo.currentText() == "Staggered Walls"
        is_maze_mode = self.map_mode_combo.currentText() == "Perfect Maze"

        self.map_file_edit.setEnabled(is_file_mode)
        self.map_browse_button.setEnabled(is_file_mode)

        self.wall_probability_spin.setEnabled(is_random_mode or is_staggered_mode)
        self.random_width_spin.setEnabled(is_random_mode or is_staggered_mode or is_maze_mode)
        self.random_height_spin.setEnabled(is_random_mode or is_staggered_mode or is_maze_mode)
        if is_staggered_mode:
            self.wall_parameter_label.setText("Barrier density:")
        else:
            self.wall_parameter_label.setText("Wall p:")
        self._update_endpoint_ranges()

    def _update_endpoint_ranges(self) -> None:
        mode = self.map_mode_combo.currentText()
        if mode == "File Map":
            if self._payload and self._payload.get("map"):
                height = len(self._payload["map"]["grid"])
                width = len(self._payload["map"]["grid"][0]) if height else 1
            else:
                width = 9999
                height = 9999
        else:
            width = max(1, self.random_width_spin.value())
            height = max(1, self.random_height_spin.value())

        for spin_box, upper_bound in (
            (self.start_row_spin, height - 1),
            (self.goal_row_spin, height - 1),
            (self.start_col_spin, width - 1),
            (self.goal_col_spin, width - 1),
        ):
            spin_box.setMaximum(max(0, upper_bound))

    def _selected_algorithm_tokens(self) -> list[str]:
        mapping = {
            "BFS": "bfs",
            "A*": "astar",
            "Dijkstra": "dijkstra",
            "B* PaperCrawl": "bstar_paper",
            "B* GreedyLite": "bstar_greedy_lite",
            "B* Robust": "bstar_robust",
            "IBP-B*": "ibp_bstar",
        }
        selected = [mapping[label] for label, checkbox in self.algorithm_checkboxes.items() if checkbox.isChecked()]
        return selected

    def _build_command(self) -> list[str]:
        executable = Path(self.exe_path_edit.text().strip())
        if not executable.exists():
            raise RuntimeError("The selected executable does not exist.")

        command = [str(executable), "--json"]

        mode = self.map_mode_combo.currentText()
        if mode == "File Map":
            map_path = self.map_file_edit.text().strip()
            if not map_path:
                raise RuntimeError("Please choose a map file.")
            command.extend(["--map", map_path])
        elif mode == "Random Map":
            command.extend(
                [
                    "--random",
                    str(self.random_width_spin.value()),
                    str(self.random_height_spin.value()),
                    str(self.wall_probability_spin.value()),
                ]
            )
            command.extend(["--seed", str(self.seed_spin.value())])
        elif mode == "Staggered Walls":
            command.extend(
                [
                    "--staggered-walls",
                    str(self.random_width_spin.value()),
                    str(self.random_height_spin.value()),
                    str(self.wall_probability_spin.value()),
                ]
            )
            command.extend(["--seed", str(self.seed_spin.value())])
        else:
            command.extend(["--maze", str(self.random_width_spin.value()), str(self.random_height_spin.value())])
            command.extend(["--seed", str(self.seed_spin.value())])

        algorithm_tokens = self._selected_algorithm_tokens()
        if not algorithm_tokens:
            raise RuntimeError("Please select at least one algorithm.")
        command.extend(["--algorithms", ",".join(algorithm_tokens)])

        if self.branch_no_reverse_checkbox.isChecked():
            command.append("--branch-no-reverse")

        command.extend(["--ibp-wait", str(self.ibp_wait_spin.value())])
        command.extend(["--ibp-zigzag-threshold", str(self.ibp_zigzag_threshold_spin.value())])

        if self.ibp_paper_strict_checkbox.isChecked():
            command.append("--ibp-paper-strict")

        if self.start_override_checkbox.isChecked():
            command.extend(["--sx", str(self.start_row_spin.value()), "--sy", str(self.start_col_spin.value())])

        if self.goal_override_checkbox.isChecked():
            command.extend(["--ex", str(self.goal_row_spin.value()), "--ey", str(self.goal_col_spin.value())])

        return command

    def _run_comparison(self) -> None:
        try:
            command = self._build_command()
        except RuntimeError as error:
            QtWidgets.QMessageBox.warning(self, "Invalid Configuration", str(error))
            return

        self.run_button.setEnabled(False)
        self.statusBar().showMessage("Running comparison...")
        self._set_output_text("$ " + " ".join(command) + "\n")

        self._worker_thread = QtCore.QThread(self)
        self._worker = CompareWorker(command, self._workspace_root)
        self._worker.moveToThread(self._worker_thread)

        self._worker_thread.started.connect(self._worker.run)
        self._worker.finished.connect(self._on_worker_finished)
        self._worker.failed.connect(self._on_worker_failed)
        self._worker.done.connect(self._worker_thread.quit)
        self._worker.done.connect(self._worker.deleteLater)
        self._worker_thread.finished.connect(self._worker_thread.deleteLater)
        self._worker_thread.finished.connect(lambda: self.run_button.setEnabled(True))
        self._worker_thread.finished.connect(lambda: self.statusBar().showMessage("Ready"))
        self._worker_thread.start()

    def _on_worker_finished(self, payload: dict, raw_output: str) -> None:
        self._payload = payload
        self.canvas.set_payload(payload)
        self.canvas.set_overlay_all_paths(self.overlay_paths_checkbox.isChecked())
        self._update_endpoint_ranges()
        map_info = payload.get("map", {})
        start_point = map_info.get("start", {})
        goal_point = map_info.get("goal", {})
        if not self.start_override_checkbox.isChecked():
            self.start_row_spin.setValue(int(start_point.get("row", 0)))
            self.start_col_spin.setValue(int(start_point.get("col", 0)))
        if not self.goal_override_checkbox.isChecked():
            self.goal_row_spin.setValue(int(goal_point.get("row", 0)))
            self.goal_col_spin.setValue(int(goal_point.get("col", 0)))
        self._set_output_text(raw_output)
        self._populate_results(payload.get("algorithms", []))
        self.statusBar().showMessage("Comparison finished successfully.")

    def _on_worker_failed(self, message: str, raw_output: str) -> None:
        self._payload = None
        self.canvas.set_payload(None)
        self._update_endpoint_ranges()
        self.results_table.setRowCount(0)
        self.algorithm_combo.clear()
        self._set_output_text(raw_output or message)
        QtWidgets.QMessageBox.critical(self, "Comparison Failed", message)
        self.statusBar().showMessage("Comparison failed.")

    def _set_output_text(self, text: str) -> None:
        max_length = 200_000
        if len(text) > max_length:
            text = text[:max_length] + "\n\n... output truncated for readability ..."
        self.output_text.setPlainText(text)

    def _populate_results(self, algorithms: list[dict]) -> None:
        self.results_table.setRowCount(len(algorithms))
        self.algorithm_combo.blockSignals(True)
        self.results_table.blockSignals(True)
        self.algorithm_combo.clear()

        first_algorithm_name = ""
        for row, algorithm in enumerate(algorithms):
            name = str(algorithm.get("name", "Unknown"))
            if not first_algorithm_name:
                first_algorithm_name = name
            values = [
                name,
                "yes" if algorithm.get("success") else "no",
                str(algorithm.get("path_length", 0)),
                str(algorithm.get("expanded", 0)),
                str(algorithm.get("time_us", 0)),
                "-" if algorithm.get("gap_to_bfs") is None else str(algorithm.get("gap_to_bfs")),
            ]

            for column, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                if column == 0:
                    item.setForeground(QtGui.QBrush(self.canvas._path_colors.get(name, QtGui.QColor("#334e68"))))
                self.results_table.setItem(row, column, item)

            self.algorithm_combo.addItem(name)

        self.algorithm_combo.blockSignals(False)
        self.results_table.blockSignals(False)
        if first_algorithm_name:
            self.algorithm_combo.setCurrentText(first_algorithm_name)
            self.canvas.set_selected_algorithm_name(first_algorithm_name)
            self.results_table.selectRow(0)

    def _on_algorithm_changed(self, algorithm_name: str) -> None:
        self.canvas.set_selected_algorithm_name(algorithm_name)
        self._sync_table_selection(algorithm_name)

    def _on_overlay_toggled(self, enabled: bool) -> None:
        self.canvas.set_overlay_all_paths(enabled)

    def _on_table_selection_changed(self) -> None:
        selected_items = self.results_table.selectedItems()
        if not selected_items:
            return
        row = selected_items[0].row()
        name_item = self.results_table.item(row, 0)
        if not name_item:
            return
        algorithm_name = name_item.text()
        if self.algorithm_combo.currentText() == algorithm_name:
            return
        self.algorithm_combo.blockSignals(True)
        self.algorithm_combo.setCurrentText(algorithm_name)
        self.algorithm_combo.blockSignals(False)
        self.canvas.set_selected_algorithm_name(algorithm_name)

    def _sync_table_selection(self, algorithm_name: str) -> None:
        self.results_table.blockSignals(True)
        try:
            for row in range(self.results_table.rowCount()):
                item = self.results_table.item(row, 0)
                if item and item.text() == algorithm_name:
                    self.results_table.selectRow(row)
                    break
        finally:
            self.results_table.blockSignals(False)

    def _zoom_in(self) -> None:
        self.canvas.zoom_in()

    def _zoom_out(self) -> None:
        self.canvas.zoom_out()

    def _reset_zoom(self) -> None:
        self.canvas.reset_zoom()

    def _set_pick_mode(self, target: str, enabled: bool) -> None:
        if enabled:
            self._pending_pick_target = target
            other_button = self.pick_goal_button if target == "start" else self.pick_start_button
            other_button.blockSignals(True)
            other_button.setChecked(False)
            other_button.blockSignals(False)
            self.statusBar().showMessage(f"Click a cell on the map to set the {target}.")
            return

        if self._pending_pick_target == target:
            self._pending_pick_target = None
            self.statusBar().showMessage("Ready")

    def _on_canvas_cell_clicked(self, row: int, col: int) -> None:
        if self._payload and self._payload.get("map"):
            grid = self._payload["map"]["grid"]
            if 0 <= row < len(grid) and 0 <= col < len(grid[0]) and grid[row][col] == 1:
                self.statusBar().showMessage(f"Cell ({row}, {col}) is blocked. Please pick a free cell.")
                return

        if self._pending_pick_target == "start":
            self.start_override_checkbox.setChecked(True)
            self.start_row_spin.setValue(row)
            self.start_col_spin.setValue(col)
            self.pick_start_button.blockSignals(True)
            self.pick_start_button.setChecked(False)
            self.pick_start_button.blockSignals(False)
            self._pending_pick_target = None
            self.statusBar().showMessage(f"Start override set to ({row}, {col}).")
            return

        if self._pending_pick_target == "goal":
            self.goal_override_checkbox.setChecked(True)
            self.goal_row_spin.setValue(row)
            self.goal_col_spin.setValue(col)
            self.pick_goal_button.blockSignals(True)
            self.pick_goal_button.setChecked(False)
            self.pick_goal_button.blockSignals(False)
            self._pending_pick_target = None
            self.statusBar().showMessage(f"Goal override set to ({row}, {col}).")
            return

        self.statusBar().showMessage(f"Cell ({row}, {col})")


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("Pathfinding Compare Viewer")

    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
