"""
Модуль для управления обработкой седел клапанов на станке с ЧПУ.
Интегрируется с LinuxCNC через HAL и предоставляет GUI на основе GTK.
"""
import hal
import math
import os
import json
import sys
import linuxcnc
import numpy as np
import pandas as pd
from gi.repository import Gtk
from matplotlib.figure import Figure
from matplotlib.backends.backend_gtk3agg import FigureCanvasGTK3Agg as FigureCanvas
from shapely.geometry import LineString, Point
from typing import List, Tuple
from dataclasses import dataclass

# =============================================================================
# КОНСТАНТЫ
# =============================================================================
HOME_DIR = os.path.expanduser("~/") + "/linuxcnc/project/valve_head/"
TABLE_HEADERS: List[str] = ["№", "X", "Y", "Тип", "Коэфф. кривизны", "Примечание"]
Z_SAFE_HEIGHT: float = 20.0
VALVE_TYPE_INTAKE: str = "впускной"
VALVE_TYPE_EXHAUST: str = "выпускной"


# =============================================================================
# КЛАССЫ ДАННЫХ
# =============================================================================
@dataclass
class ValveParameters:
    """Параметры клапана для обработки."""
    fd: float       # Диаметр заготовки
    vsd: float      # Глубина седла
    vsdtr_1: float  # Диаметр перехода (заготовка)
    vsdtr_2: float  # Диаметр перехода (седло)
    vsa_1: float    # Угол седла (заготовка)
    vsa_2: float    # Угол седла (седло)
    vsw_1: float    # Ширина седла (заготовка)
    vsw_2: float    # Ширина седла (седло)
    vsa2_1: float   # Дополнительный угол (заготовка)
    vsa2_2: float   # Дополнительный угол (седло)
    ff: float       # Смещение (сколько снять)
    md: float       # Внутренний диаметр заготовки


# =============================================================================
# ВСПОМОГАТЕЛЬНЫЕ КЛАССЫ
# =============================================================================
class LinuxCNCHelper:
    """Класс-обертка для безопасной работы с LinuxCNC."""

    def __init__(self):
        self._command = linuxcnc.command()
        self._stat = linuxcnc.stat()

    def get_position(self) -> Tuple[float, ...]:
        try:
            self._stat.poll()
            return self._stat.position
        except linuxcnc.error as detail:
            print(f"Ошибка LinuxCNC: {detail}")
            sys.exit(1)

    def get_xy(self) -> Tuple[float, float]:
        pos = self.get_position()
        return round(pos[0], 2), round(pos[1], 2)

    def get_z(self) -> float:
        pos = self.get_position()
        return round(pos[2], 2)

    def send_mdi(self, msg: str) -> None:
        self._command.mode(linuxcnc.MODE_MDI)
        self._command.wait_complete()
        self._command.mdi(f"(MSG, {msg})")

    def run_program(self, filepath: str, on_complete_callback=None) -> None:
        self._command.reset_interpreter()
        self._command.program_open(filepath)
        self._command.wait_complete(100)
        self._command.mode(linuxcnc.MODE_AUTO)
        self._command.auto(linuxcnc.AUTO_RUN, 0)

        while True:
            self._stat.poll()
            if self._stat.interp_state == linuxcnc.INTERP_IDLE:
                if on_complete_callback:
                    on_complete_callback()
                break
            self._command.wait_complete(10)


class ValveSeatContour:
    """Класс для представления контура седла клапана."""

    def __init__(self, fd: float, vsdtr: float, vsa: float, vsw: float,
                 vsa2: float, vsd: float, y_offset: float, md: float):
        self.fd = fd
        self.vsdtr = vsdtr
        self.vsa = vsa
        self.vsw = vsw
        self.vsa2 = vsa2
        self.vsd = vsd
        self.y_offset = y_offset
        self.md = md

    def get_breakpoints(self) -> List[Tuple[float, float]]:
        """Возвращает точки перегиба траектории в формате [(x1, y1), ...]."""
        vsa_rad = math.radians(self.vsa)
        vsa2_rad = math.radians(self.vsa2)

        x0, y0 = -self.fd / 2, self.y_offset
        x1, y1 = -self.vsdtr / 2, self.y_offset
        x2 = -self.vsdtr / 2 + self.vsw * math.cos(vsa_rad)
        y2 = -self.vsw * math.sin(vsa_rad) + self.y_offset
        x3 = x2 + (self.vsd - self.vsw * math.sin(vsa_rad)) / math.tan(vsa2_rad)
        y3 = min(-self.vsd + self.y_offset, 0)

        return [(x0, y0), (x1, y1), (x2, y2), (x3, y3)]


# =============================================================================
# ОСНОВНОЙ ОБРАБОТЧИК GUI
# =============================================================================
class HandlerClass:
    """Основной класс обработчика GUI для управления обработкой седел клапанов."""

    # Ключи JSON и соответствующие им ID виджетов (порядок совпадает)
    _PARAM_KEYS: List[str] = [
        'f', 'rpm', 'fpp',
        'fd_in', 'vsd_in', 'vsdtr_1_in', 'vsdtr_2_in', 'vsa_1_in', 'vsa_2_in',
        'vsw_1_in', 'vsw_2_in', 'vsa2_1_in', 'vsa2_2_in', 'ff_in', 'md_in',
        'fd_out', 'vsd_out', 'vsdtr_1_out', 'vsdtr_2_out', 'vsa_1_out', 'vsa_2_out',
        'vsw_1_out', 'vsw_2_out', 'vsa2_1_out', 'vsa2_2_out', 'ff_out', 'md_out',
    ]
    _ENTRY_IDS: List[str] = [
        "EntryF", "EntryRPM", "EntryFPP",
        "EntryFD_in", "EntryVSD_in", "EntryVSDtr1_in", "EntryVSDtr2_in",
        "EntryVSA1_in", "EntryVSA2_in", "EntryVSW1_in", "EntryVSW2_in",
        "EntryVSA21_in", "EntryVSA22_in", "EntryFF_in", "EntryMD_in",
        "EntryFD_out", "EntryVSD_out", "EntryVSDtr1_out", "EntryVSDtr2_out",
        "EntryVSA1_out", "EntryVSA2_out", "EntryVSW1_out", "EntryVSW2_out",
        "EntryVSA21_out", "EntryVSA22_out", "EntryFF_out", "EntryMD_out",
    ]

    def __init__(self, halcomp, builder, useropts):
        self.halcomp = halcomp
        self.builder = builder
        self.nhits = 0
        self.canvas = None
        self.table_data = []
        self.z0_out = 0
        self.z0_in = 0
        self.x = 0
        self.y = 0
        self.cnc = LinuxCNCHelper()

        self.textview = self.builder.get_object("prog")
        if self.textview:
            self.textbuffer = self.textview.get_buffer()

    # ------------------------------------------------------------------
    # Вспомогательные методы
    # ------------------------------------------------------------------

    def file_path(self, file_name: str) -> str:
        """Возвращает полный путь к файлу в директории проекта."""
        name_pr = self.builder.get_object("name_project").get_text()
        path = os.path.join(HOME_DIR, name_pr, file_name)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        return path

    def get_entry_value(self, entry_name: str, default: float = 0.0) -> float:
        """Получает числовое значение из поля ввода GUI."""
        entry = self.builder.get_object(entry_name)
        if not entry:
            return default
        try:
            return float(entry.get_text().replace(',', '.'))
        except ValueError:
            return default

    def message_mdi(self, msg: str) -> None:
        self.cnc.send_mdi(msg)

    def get_xy(self) -> Tuple[float, float]:
        return self.cnc.get_xy()

    def get_z(self) -> float:
        return self.cnc.get_z()

    # ------------------------------------------------------------------
    # Поиск центра
    # ------------------------------------------------------------------

    def get_current_pilot(self) -> Tuple[float, float, float, float, float]:
        return (
            self.get_entry_value("f_slow"),
            self.get_entry_value("f_fast"),
            self.get_entry_value("d_seat"),
            self.get_entry_value("h_probe_depth"),
            self.get_entry_value("probe_retract"),
        )

    def _create_hole_program(self, f_slow, f_fast, d_seat, h_probe_depth, probe_retract,
                              d_bush, h_bush) -> str:
        """Двухэтапный поиск центра:
        Этап 1 — центр седла: щуп опускается на h_probe_depth и 4 раза касается стенок
                  отверстия седла изнутри (d_seat). Z поднимается, щуп встаёт в центр.
        Этап 2 — центр втулки: щуп опускается на h_bush, суброутина o100 зондирует
                  стенки втулки (d_bush) дважды с разворотом на 180°, результат усредняется.
        """
        so_s = d_seat / 2 - 2    # смещение от центра к стенке седла
        pd_s = d_seat - 4        # дистанция зондирования седла (с запасом)
        so_b = d_bush / 2 - 2    # смещение от центра к стенке втулки
        pd_b = d_bush - 4        # дистанция зондирования втулки (с запасом)
        pr = probe_retract

        # Суброутина o100: 4-стороннее зондирование втулки на текущей глубине Z.
        # Сохраняет текущий центр в #35/#36, результаты стенок — в #31-#34.
        bush_sub = (
            f'o100 sub\n'
            f'#35 = #<_x>\n'
            f'#36 = #<_y>\n'
            f'G91\n'
            f'G0 X-{so_b}\n'
            f'G38.2 X-{pd_b} F{f_fast}\n'
            f'G38.4 X{pr*1.2} F{f_slow}\n'
            f'G38.2 X-{pr*3} F{f_slow}\n'
            f'#31 = #<_x>\n'
            f'G0 X{pr*2}\n'
            f'G90\nG0 X#35\nG91\n'
            f'G0 X{so_b}\n'
            f'G38.2 X{pd_b} F{f_fast}\n'
            f'G38.4 X-{pr*1.2} F{f_slow}\n'
            f'G38.2 X{pr*3} F{f_slow}\n'
            f'#32 = #<_x>\n'
            f'G0 X-{pr*2}\n'
            f'G90\nG0 X#35\nG0 Y#36\nG91\n'
            f'G0 Y-{so_b}\n'
            f'G38.2 Y-{pd_b} F{f_fast}\n'
            f'G38.4 Y{pr*1.2} F{f_slow}\n'
            f'G38.2 Y-{pr*3} F{f_slow}\n'
            f'#33 = #<_y>\n'
            f'G0 Y{pr*2}\n'
            f'G90\nG0 Y#36\nG91\n'
            f'G0 Y{so_b}\n'
            f'G38.2 Y{pd_b} F{f_fast}\n'
            f'G38.4 Y-{pr*1.2} F{f_slow}\n'
            f'G38.2 Y{pr*3} F{f_slow}\n'
            f'#34 = #<_y>\n'
            f'G0 Y-{pr*2}\n'
            f'G90\nG0 X#35\nG0 Y#36\n'
            f'o100 endsub\n\n'
        )

        # Этап 1: поиск центра седла (однократно, изнутри отверстия)
        seat_part = (
            f'(MSG, Поиск центра отверстия седла)\n'
            f'G54 G90\n'
            f'G21 G91\n'
            f'#5 = #<_x>\n'
            f'#6 = #<_y>\n'
            f'G0 Z-{h_probe_depth}\n'
            # левая стенка
            f'G0 X-{so_s}\n'
            f'G38.2 X-{pd_s} F{f_fast}\n'
            f'G38.4 X{pr*1.2} F{f_slow}\n'
            f'G38.2 X-{pr*3} F{f_slow}\n'
            f'#1 = #<_x>\n'
            f'G0 X{pr*2}\n'
            f'G90\nG0 X#5\nG91\n'
            # правая стенка
            f'G0 X{so_s}\n'
            f'G38.2 X{pd_s} F{f_fast}\n'
            f'G38.4 X-{pr*1.2} F{f_slow}\n'
            f'G38.2 X{pr*3} F{f_slow}\n'
            f'#2 = #<_x>\n'
            f'G0 X-{pr*2}\n'
            f'G90\nG0 X#5\nG0 Y#6\nG91\n'
            # нижняя стенка
            f'G0 Y-{so_s}\n'
            f'G38.2 Y-{pd_s} F{f_fast}\n'
            f'G38.4 Y{pr*1.2} F{f_slow}\n'
            f'G38.2 Y-{pr*3} F{f_slow}\n'
            f'#3 = #<_y>\n'
            f'G0 Y{pr*2}\n'
            f'G90\nG0 Y#6\nG91\n'
            # верхняя стенка
            f'G0 Y{so_s}\n'
            f'G38.2 Y{pd_s} F{f_fast}\n'
            f'G38.4 Y-{pr*1.2} F{f_slow}\n'
            f'G38.2 Y{pr*3} F{f_slow}\n'
            f'#4 = #<_y>\n'
            f'G0 Y-{pr*2}\n'
            # подъём и перемещение в центр седла
            f'G0 Z{h_probe_depth}\n'
            f'G90\n'
            f'G0 X[ [#1 + #2] / 2 ] Y[ [#3 + #4] / 2 ]\n'
        )

        # Этап 2: поиск центра втулки (два прохода с разворотом на 180°)
        bush_part = (
            f'G91\n'
            f'G0 Z-{h_bush}\n'
            f'o100 call\n'
            f'#41 = #31\n#42 = #32\n#43 = #33\n#44 = #34\n'
            f'G0 Z{h_bush}\n'
            f'G90\n'
            f'G0 X[ [#41 + #42] / 2 ] Y[ [#43 + #44] / 2 ]\n'
            f'(MSG, Разверните щуп на 180 градусов)\n'
            f'M0\n'
            f'G91\n'
            f'G0 Z-{h_bush}\n'
            f'o100 call\n'
            f'G0 Z{h_bush}\n'
            f'G90\n'
            f'G0 X[ [#41 + #42 + #31 + #32] / 4 ] Y[ [#43 + #44 + #33 + #34] / 4 ]\n'
            f'M2'
        )

        return bush_sub + seat_part + bush_part

    def get_current_bush_params(self) -> Tuple[float, float]:
        return (
            self.get_entry_value("d_bush"),
            self.get_entry_value("h_bush"),
        )

    def create_program(self) -> None:
        f_slow, f_fast, d_seat, h_probe_depth, probe_retract = self.get_current_pilot()
        d_bush, h_bush = self.get_current_bush_params()
        program = self._create_hole_program(
            f_slow, f_fast, d_seat, h_probe_depth, probe_retract, d_bush, h_bush)
        with open(self.file_path("centr_pr.ngc"), "w", encoding="utf-8") as f:
            f.write(program)

    def find_center(self, widget) -> None:
        self.create_program()
        self.cnc.run_program(
            self.file_path("centr_pr.ngc"),
            on_complete_callback=lambda: self.message_mdi("Программа завершена."),
        )
        coord = self.cnc.get_xy()
        self.x, self.y = coord
        label = self.builder.get_object("val_centre")
        if label:
            label.set_text(f"X{self.x} Y{self.y}")
        else:
            self.message_mdi("Label 'val_centre' не найден!")

    # ------------------------------------------------------------------
    # Таблица центров клапанов
    # ------------------------------------------------------------------

    def _get_valve_type(self) -> str:
        valve_in_btn = self.builder.get_object("valv_in")
        if valve_in_btn and valve_in_btn.get_active():
            return VALVE_TYPE_INTAKE
        valve_out_btn = self.builder.get_object("valv_out")
        if valve_out_btn and valve_out_btn.get_active():
            return VALVE_TYPE_EXHAUST
        return "неизвестно"

    def _clear_input_fields(self) -> None:
        for field_name in ["val_num", "curva_koef", "comment"]:
            entry = self.builder.get_object(field_name)
            if entry:
                entry.set_text("0.0" if field_name == "curva_koef" else "")

    def _build_table_row(self, val_num_tab: int) -> List:
        coord = self.cnc.get_xy()
        return [
            val_num_tab,
            f"{coord[0]:.2f}",
            f"{coord[1]:.2f}",
            self._get_valve_type(),
            self.get_entry_value("curva_koef"),
            self.builder.get_object("comment").get_text(),
        ]

    def _upsert_table_row(self, row: List) -> None:
        """Обновляет строку с совпадающим номером или добавляет новую."""
        num = int(row[0])
        for i, existing in enumerate(self.table_data):
            if int(existing[0]) == num:
                self.table_data[i] = row
                return
        self.table_data.append(row)

    def _update_table_ui(self) -> None:
        """Перестраивает GtkGrid с таблицей и сохраняет данные в CSV."""
        grid = Gtk.Grid()
        grid.set_column_spacing(10)
        grid.set_row_spacing(5)

        for col, title in enumerate(TABLE_HEADERS):
            label = Gtk.Label(label=f"<b>{title}</b>", use_markup=True)
            grid.attach(label, col, 0, 1, 1)

        for row_index, row_data in enumerate(self.table_data, start=1):
            for col_index, cell in enumerate(row_data):
                label = Gtk.Label(label=str(cell))
                grid.attach(label, col_index, row_index, 1, 1)

        frame = self.builder.get_object("table_centers")
        for child in frame.get_children():
            frame.remove(child)
        frame.add(grid)
        frame.show_all()

        if self.table_data:
            pd.DataFrame(self.table_data).to_csv(
                self.file_path("table_center.csv"), index=False, header=False
            )

    def add_table(self, widget) -> None:
        val_num = self.get_entry_value("val_num")
        val_num_tab = int(val_num) if val_num != 0 else len(self.table_data) + 1
        self._upsert_table_row(self._build_table_row(val_num_tab))
        self._clear_input_fields()
        self._update_table_ui()

    # add_table и change_table идентичны по поведению
    change_table = add_table

    def remove_row(self, widget) -> None:
        if self.table_data:
            self.table_data.pop()
        self._update_table_ui()

    def load_table(self, widget) -> None:
        f_name = self.builder.get_object("file_table")
        filename = f_name.get_filename() or self.file_path("table_center.csv")
        try:
            data = pd.read_csv(filename, header=None)
            self.table_data = data.astype(str).values.tolist()
            self._update_table_ui()
        except Exception as e:
            self.message_mdi(f"Ошибка при загрузке таблицы: {e}")

    # ------------------------------------------------------------------
    # Параметры клапанов
    # ------------------------------------------------------------------

    def get_current_values(self) -> Tuple[float, float, float]:
        return (
            self.get_entry_value("EntryF"),
            self.get_entry_value("EntryRPM"),
            self.get_entry_value("EntryFPP"),
        )

    def _get_valve_params(self, suffix: str) -> ValveParameters:
        g = lambda name: self.get_entry_value(name)
        return ValveParameters(
            fd=g(f"EntryFD_{suffix}"),
            vsd=g(f"EntryVSD_{suffix}"),
            vsdtr_1=g(f"EntryVSDtr1_{suffix}"),
            vsdtr_2=g(f"EntryVSDtr2_{suffix}"),
            vsa_1=g(f"EntryVSA1_{suffix}"),
            vsa_2=g(f"EntryVSA2_{suffix}"),
            vsw_1=g(f"EntryVSW1_{suffix}"),
            vsw_2=g(f"EntryVSW2_{suffix}"),
            vsa2_1=g(f"EntryVSA21_{suffix}"),
            vsa2_2=g(f"EntryVSA22_{suffix}"),
            ff=g(f"EntryFF_{suffix}"),
            md=g(f"EntryMD_{suffix}"),
        )

    def get_current_values_in(self) -> ValveParameters:
        return self._get_valve_params("in")

    def get_current_values_out(self) -> ValveParameters:
        return self._get_valve_params("out")

    def _make_valve_contours(self, p: ValveParameters, fpp: float):
        """Создаёт контуры заготовки, седла и список проходов обработки."""
        blank = ValveSeatContour(fd=p.fd, vsdtr=p.vsdtr_1, vsa=p.vsa_1, vsw=p.vsw_1,
                                 vsa2=p.vsa2_1, vsd=p.vsd, y_offset=0, md=p.md)
        final = ValveSeatContour(fd=p.fd, vsdtr=p.vsdtr_2, vsa=p.vsa_2, vsw=p.vsw_2,
                                 vsa2=p.vsa2_2, vsd=p.vsd, y_offset=-p.ff, md=p.md)
        processing = [
            ValveSeatContour(fd=p.fd, vsdtr=p.vsdtr_2, vsa=p.vsa_2, vsw=p.vsw_2,
                             vsa2=p.vsa2_2, vsd=p.vsd, y_offset=y, md=p.md)
            for y in np.arange(-p.ff, p.vsd, fpp)
        ]
        return blank, final, processing, p.md

    def create_valve_objects(self):
        _, _, fpp = self.get_current_values()
        blank_in, valve_in, proc_in, md_in = self._make_valve_contours(self.get_current_values_in(), fpp)
        blank_out, valve_out, proc_out, md_out = self._make_valve_contours(self.get_current_values_out(), fpp)
        return blank_in, valve_in, proc_in, md_in, blank_out, valve_out, proc_out, md_out

    # ------------------------------------------------------------------
    # Геометрические проверки
    # ------------------------------------------------------------------

    def check_polylines_intersect(self, bv_breakpoints, proc_valve) -> bool:
        return LineString(bv_breakpoints).intersects(LineString(proc_valve))

    def check_polylines_interpolate(self, bv_breakpoints, proc_valve) -> bool:
        line1 = LineString(bv_breakpoints)
        return all(
            p2[1] > line1.interpolate(line1.project(Point(p2))).y
            for p2 in LineString(proc_valve).coords
        )

    def valve_proc(self, valve_processing, blank_valve, md) -> List:
        clean_proc = []
        bv_breakpoints_b = blank_valve.get_breakpoints()

        for proc_valve in valve_processing:
            break_x_proc, break_y_proc = zip(*proc_valve.get_breakpoints())
            break_x, break_y = list(break_x_proc), list(break_y_proc)

            # Обрезаем по внутреннему диаметру заготовки
            if break_x[3] >= -md / 2:
                t = (-md / 2 - break_x[2]) / (break_x[3] - break_x[2])
                break_y[3] = break_y[2] + t * (break_y[3] - break_y[2])
                break_x[3] = -md / 2
            else:
                break_x.append(-md / 2)
                break_y.append(break_y[3])

            pts = list(zip(break_x, break_y))
            above_up = self.check_polylines_interpolate(bv_breakpoints_b, pts)
            above = self.check_polylines_intersect(bv_breakpoints_b, pts)

            if (not above) and above_up:
                break

            # Обрезаем по Z=0; break_y_proc — оригинальные значения до модификации
            for i in range(len(break_y_proc)):
                if break_y_proc[i] > 0:
                    if break_y[i + 1] - break_y[i] != 0:
                        break_x[i] += (0 - break_y[i]) * (break_x[i + 1] - break_x[i]) / (break_y[i + 1] - break_y[i])
                    break_y[i] = 0

            # Убираем дублирующиеся точки на Z=0
            for i in range(len(break_y) - 1, 0, -1):
                if break_y[i] == 0 and break_y[i - 1] == 0:
                    break_x[i - 1] = break_x[i]

            clean_proc.append(list(zip(break_x, break_y)))

        return [s[::-1] for s in clean_proc[::-1]]

    # ------------------------------------------------------------------
    # Генерация G-кода
    # ------------------------------------------------------------------

    def valve_gcode_prefix(self) -> str:
        _, rpm, _ = self.get_current_values()
        return (
            f'G21  ; Используем миллиметры\n'
            f'G90  ; Абсолютные координаты\n'
            f'M3 S{rpm} ; Старт шпинделя\n'
        )

    def valve_gcode_suffix(self) -> str:
        return (
            f'G53 G0 Z{Z_SAFE_HEIGHT + self.z0_in} ; Поднимаем Z\n'
            f'M30  ; Конец программы\n'
        )

    def prog_valve_seat(self, clean_proc: List, md: float, start: int = 0) -> str:
        if start > len(clean_proc):
            self.message_mdi('Не корректный номер прохода для начала')
            return ''

        f, _, _ = self.get_current_values()
        program = ''
        prev = ''

        for i, pass_points in enumerate(clean_proc[start:], start=start):
            for j, point in enumerate(pass_points):
                cmd = (
                    f'G0 U{point[0]:.2f} Z{point[1]:.2f} F{f} ; Проход {i}\n'
                    if j == 0 else
                    f'G1 U{point[0]:.2f} Z{point[1]:.2f} F{f} ; Проход {i}\n'
                )
                if cmd != prev:
                    program += cmd
                    prev = cmd

            if i != len(clean_proc) - 1:
                program += f'G0 U{-md/2} Z{point[1]:.2f} F{f} ; Конец прохода {i}\n'

        end_x = clean_proc[-1][-1][0]
        start_x = clean_proc[0][0][0]
        program += f'G1 U{end_x:.3f} Z0  ; Обрабатываем кромку\n'
        program += f'G0 U{start_x:.3f} Z0  ; Возвращаемся в исходное положение\n'
        return program

    # ------------------------------------------------------------------
    # Визуализация
    # ------------------------------------------------------------------

    def _draw_valve_seat_common(self, container_name: str, blank_valve,
                                valve, valve_processing, md) -> None:
        container = self.builder.get_object(container_name)
        for child in container.get_children():
            container.remove(child)

        fig = Figure(figsize=(9, 9), dpi=100)
        ax = fig.add_subplot(111)

        x_b, y_b = zip(*blank_valve.get_breakpoints())
        ax.plot(x_b, y_b, label="Контур заготовки", color="blue")

        x_v, y_v = zip(*valve.get_breakpoints())
        ax.plot(x_v, y_v, label="Контур клапана", color="orange")

        clean_proc = self.valve_proc(valve_processing, blank_valve, md)
        for pass_pts in clean_proc:
            x, y = zip(*pass_pts)
            ax.scatter(x, y, color="green", zorder=1, s=5)
            ax.plot(x, y, color="green", lw=0.5, linestyle="--")

        ax.axhline(0, color="gray", linestyle="--", label="Z0")
        ax.axvline(-md / 2, color="brown", linestyle="--", label="Внутренний диаметр заготовки")
        ax.legend()
        ax.set_xlabel("U")
        ax.set_ylabel("Z")
        ax.set_aspect('equal', adjustable='datalim')
        ax.axis("equal")
        ax.grid()

        self.canvas = FigureCanvas(fig)
        container.add(self.canvas)
        container.show_all()

    def draw_valve_seat_in(self) -> None:
        blank, valve, processing, md, _, _, _, _ = self.create_valve_objects()
        self._draw_valve_seat_common("proc_kontur_box", blank, valve, processing, md)

    def draw_valve_seat_out(self) -> None:
        _, _, _, _, blank, valve, processing, md = self.create_valve_objects()
        self._draw_valve_seat_common("proc_kontur_box2", blank, valve, processing, md)

    def on_draw_valve_seat_clicked(self, widget) -> None:
        self.nhits += 1
        hits_label = self.builder.get_object('hits')
        if hits_label:
            hits_label.set_label(f"Hits: {self.nhits}")

        container = self.builder.get_object("proc_kontur_box")
        if not container:
            return

        if self.canvas:
            container.remove(self.canvas)
            self.canvas = None

        self.draw_valve_seat_in()
        self.draw_valve_seat_out()

    # ------------------------------------------------------------------
    # Сохранение / загрузка параметров
    # ------------------------------------------------------------------

    def save_valves(self, widget) -> None:
        f, rpm, fpp = self.get_current_values()
        p_in = self.get_current_values_in()
        p_out = self.get_current_values_out()

        data = dict(zip(self._PARAM_KEYS, [
            f, rpm, fpp,
            p_in.fd, p_in.vsd, p_in.vsdtr_1, p_in.vsdtr_2, p_in.vsa_1, p_in.vsa_2,
            p_in.vsw_1, p_in.vsw_2, p_in.vsa2_1, p_in.vsa2_2, p_in.ff, p_in.md,
            p_out.fd, p_out.vsd, p_out.vsdtr_1, p_out.vsdtr_2, p_out.vsa_1, p_out.vsa_2,
            p_out.vsw_1, p_out.vsw_2, p_out.vsa2_1, p_out.vsa2_2, p_out.ff, p_out.md,
        ]))
        try:
            with open(self.file_path("values.json"), "w", encoding="utf-8") as file:
                json.dump(data, file, indent=4, ensure_ascii=False)
            self.message_mdi("Значения сохранены в values.json")
        except Exception as e:
            self.message_mdi(f"Ошибка при сохранении в файл: {e}")

    def load_values(self, widget) -> None:
        f_name = self.builder.get_object("file_data")
        filename = f_name.get_filename() or self.file_path("values.json")
        try:
            with open(filename, "r", encoding="utf-8") as file:
                data = json.load(file)
            for key, entry_id in zip(self._PARAM_KEYS, self._ENTRY_IDS):
                value = data.get(key)
                if value is None:
                    continue
                entry = self.builder.get_object(entry_id)
                if entry is not None:
                    entry.set_text(str(value))
        except FileNotFoundError:
            self.message_mdi("Файл values.json не найден.")
        except Exception as e:
            self.message_mdi(f"Ошибка при загрузке из файла: {e}")

    # ------------------------------------------------------------------
    # Генерация программ обработки
    # ------------------------------------------------------------------

    def parse_to_set(self, input_str: str) -> set:
        """Преобразует строку вида "1-4" или "1, 2, 3" в множество целых чисел."""
        result = set()
        for part in input_str.replace(' ', '').split(','):
            try:
                if '-' in part:
                    start, end = map(int, part.split('-'))
                    if start <= end:
                        result.update(range(start, end + 1))
                    else:
                        self.message_mdi(f"Предупреждение: Неверный диапазон {part}. Начало больше конца.")
                else:
                    result.add(int(part))
            except ValueError:
                self.message_mdi(f"Предупреждение: Не удалось преобразовать '{part}' в целое число. Пропущено.")
            except Exception as e:
                self.message_mdi(f"Предупреждение: Ошибка при обработке '{part}': {e}")
        return result

    def set_z0_in(self, widget) -> None:
        self.z0_in = self.cnc.get_z()
        label = self.builder.get_object("z0_in_G53")
        if label:
            label.set_text(f"Z0 G53 {self.z0_in}")
        else:
            self.message_mdi("Label 'z0_in_G53' не найден!")

    def set_z0_out(self, widget) -> None:
        self.z0_out = self.cnc.get_z()
        label = self.builder.get_object("z0_out_G53")
        if label:
            label.set_text(f"Z0 G53 {self.z0_out}")
        else:
            self.message_mdi("Label 'z0_out_G53' не найден!")

    def save_one_valve_prog(self, widget) -> None:
        valve_number = int(self.get_entry_value("valve number"))
        start_pass = int(self.get_entry_value("start pass"))

        if not self.table_data:
            self.message_mdi("Таблица с координатами центров отсутствует")
            return

        table_df = pd.DataFrame(self.table_data, columns=TABLE_HEADERS)
        table_df["№"] = table_df["№"].astype(int)
        valve = table_df[table_df["№"] == valve_number]

        valve_type = valve["Тип"].iloc[0]
        if valve_type == VALVE_TYPE_INTAKE:
            blank, _, processing, md, _, _, _, _ = self.create_valve_objects()
            Z0 = self.z0_in
        elif valve_type == VALVE_TYPE_EXHAUST:
            _, _, _, _, blank, _, processing, md = self.create_valve_objects()
            Z0 = self.z0_out
        else:
            self.message_mdi(f"Неизвестный тип клапана: {valve_type}")
            return

        curva = float(valve["Коэфф. кривизны"].iloc[0])
        program = (
            f'G90  ; Абсолютные координаты\n'
            f'G53 G0 Z{Z_SAFE_HEIGHT + Z0 + curva}  ; Корректировка Z с учетом смещения и коэф. кривизны\n'
            f'G10 L20 P0 Z{Z_SAFE_HEIGHT} ; Задаем откорректированное значение Z{Z_SAFE_HEIGHT}\n'
            f'G0 U{-md/2} ; Переводим U на U{-md/2}\n'
            f'G53 G0 X{valve["X"].iloc[0]} Y{valve["Y"].iloc[0]} ; Переходим к клапану №{valve_number}\n'
            f'M0 ; Пауза\n'
        )
        clean_proc = self.valve_proc(processing, blank, md)
        program += self.prog_valve_seat(clean_proc, md, start_pass)
        program += (
            f'G90  ; Абсолютные координаты\n'
            f'G10 L20 P0 Z0 ; Обнуление Z\n'
        )

        filepath = self.file_path(f"valve_{valve_number}_start_pass{start_pass}.ngc")
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(self.valve_gcode_prefix() + program + self.valve_gcode_suffix())

    def run_and_save_in(self, widget) -> None:
        start_pass = int(self.get_entry_value("EntryFD_start_pass_in"))
        blank, _, processing, md, _, _, _, _ = self.create_valve_objects()
        clean_proc = self.valve_proc(processing, blank, md)
        program = self.valve_gcode_prefix() + self.prog_valve_seat(clean_proc, md, start_pass) + self.valve_gcode_suffix()

        filepath = self.file_path("valve_in.ngc")
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(program)
        self.cnc.run_program(filepath, on_complete_callback=lambda: self.message_mdi("Программа завершена."))

    def run_and_save_out(self, widget) -> None:
        start_pass = int(self.get_entry_value("EntryFD_start_pass_out"))
        _, _, _, _, blank, _, processing, md = self.create_valve_objects()
        clean_proc = self.valve_proc(processing, blank, md)
        program = self.valve_gcode_prefix() + self.prog_valve_seat(clean_proc, md, start_pass) + self.valve_gcode_suffix()

        filepath = self.file_path("valve_out.ngc")
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(program)
        self.cnc.run_program(filepath, on_complete_callback=lambda: self.message_mdi("Программа завершена."))

    def full_programm(self, widget) -> None:
        if not self.table_data:
            self.message_mdi("Таблица с координатами центров отсутствует")
            return

        md_in = self.get_entry_value("EntryMD_in")
        md_out = self.get_entry_value("EntryMD_out")
        start_pass_in = int(self.get_entry_value("start_pass_in"))
        start_pass_out = int(self.get_entry_value("start_pass_out"))

        valves_numer = self.parse_to_set(self.builder.get_object("valve numbers").get_text())
        self.message_mdi(f'Обрабатываем седла {valves_numer}\nДля старта обработки снимите с паузы')

        blank_in, _, proc_in, _, _, _, _, _ = self.create_valve_objects()
        clean_proc_in = self.valve_proc(proc_in, blank_in, md_in)
        program_in = self.prog_valve_seat(clean_proc_in, md_in, start_pass_in)

        _, _, _, _, blank_out, _, proc_out, _ = self.create_valve_objects()
        clean_proc_out = self.valve_proc(proc_out, blank_out, md_out)
        program_out = self.prog_valve_seat(clean_proc_out, md_out, start_pass_out)

        full = (
            'M0 ; Пауза\n'
            'o101 sub\n' + program_in + 'o101 endsub\n'
            'o102 sub\n' + program_out + 'o102 endsub\n'
        )

        for row_data in self.table_data:
            if int(row_data[0]) not in valves_numer:
                continue
            valve_type = row_data[3]
            if valve_type == VALVE_TYPE_INTAKE:
                md, Z0, sub = md_in, self.z0_in, 'o101'
            elif valve_type == VALVE_TYPE_EXHAUST:
                md, Z0, sub = md_out, self.z0_out, 'o102'
            else:
                continue
            full += (
                f'G0 Z{Z_SAFE_HEIGHT} ; Обработка седла клапана {row_data[0]} {valve_type}\n'
                f'G0 U{-md/2} ; Перемещение U на внутренний диаметр\n'
                f'G53 G0 X{row_data[1]} Y{row_data[2]} ; Перемещение к центру седла клапана {row_data[0]}\n'
                f'G53 G0 Z{Z0 + float(row_data[4])} ; Корректировка Z с учетом смещения и коэф. кривизны\n'
                f'G10 L20 P0 Z0 ; Обнуление Z\n'
                f'{sub} call ; Вызов подпрограммы обработки\n'
            )

        filepath = self.file_path("full_programm.ngc")
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(self.valve_gcode_prefix() + full + self.valve_gcode_suffix())

        self.cnc.run_program(filepath, on_complete_callback=lambda: print("Программа завершена."))


def get_handlers(halcomp, builder, useropts):
    return [HandlerClass(halcomp, builder, useropts)]
