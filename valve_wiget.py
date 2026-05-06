"""
Модуль для управления обработкой седел клапанов на станке с ЧПУ.
Интегрируется с LinuxCNC через HAL и предоставляет GUI на основе GTK.
"""
import hal
import numpy as np
import sys
import linuxcnc
from gi.repository import Gtk
import math
import pandas as pd
from matplotlib.figure import Figure
from matplotlib.backends.backend_gtk3agg import FigureCanvasGTK3Agg as FigureCanvas
from shapely.geometry import LineString, Point
from scipy.optimize import root_scalar  # type: ignore
from pathlib import Path
import os
import json
from gladevcp.core import Action
from typing import List, Tuple, Optional, Set, Dict, Any
from dataclasses import dataclass

# =============================================================================
# КОНСТАНТЫ
# =============================================================================
HOME_DIR = os.path.expanduser("~/") + "/linuxcnc/project/valve_head/"
TABLE_HEADERS: List[str] = ["№", "X", "Y", "Тип", "Коэфф. кривизны", "Примечание"]
CLEARANCE_MULTIPLIERS: Dict[str, float] = {"fast_approach": 1.2, "slow_probe": 3, "retract": 2}
PILOT_OFFSET: float = 2.0
DEFAULT_PROBE_RETRACT: float = 2.0
Z_SAFE_HEIGHT: float = 20.0

# Типы клапанов
VALVE_TYPE_INTAKE: str = "впускной"
VALVE_TYPE_EXHAUST: str = "выпускной"


# =============================================================================
# КЛАССЫ ДАННЫХ
# =============================================================================
@dataclass
class ValveParameters:
    """Параметры клапана для обработки."""
    fd: float  # Диаметр заготовки
    vsd: float  # Глубина седла
    vsdtr_1: float  # Диаметр перехода 1
    vsdtr_2: float  # Диаметр перехода 2
    vsa_1: float  # Угол седла 1
    vsa_2: float  # Угол седла 2
    vsw_1: float  # Ширина седла 1
    vsw_2: float  # Ширина седла 2
    vsa2_1: float  # Угол 2 седла 1
    vsa2_2: float  # Угол 2 седла 2
    ff: float  # Смещение
    md: float  # Внутренний диаметр


@dataclass
class ProcessingParams:
    """Параметры обработки."""
    feed: float
    rpm: int
    feed_per_pass: float


@dataclass
class ProbeParams:
    """Параметры зондирования."""
    feed_slow: float
    feed_fast: float
    seat_diameter: float
    probe_depth: float
    probe_retract: float


# =============================================================================
# ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
# =============================================================================
def save_gcode_to_file(gcode: List[str], filename: str = "cnc_program.ngc") -> None:
    """Сохраняет список команд G-кода в файл."""
    with open(filename, "w") as file:
        file.write("\n".join(gcode))
    print(f"G-код сохранен в {filename}")


class LinuxCNCHelper:
    """Класс-обертка для безопасной работы с LinuxCNC."""
    
    def __init__(self):
        self._command = linuxcnc.command()
        self._stat = linuxcnc.stat()
    
    @property
    def command(self) -> linuxcnc.command:
        return self._command
    
    @property
    def stat(self) -> linuxcnc.stat:
        return self._stat
    
    def get_position(self) -> Tuple[float, float, float]:
        """Безопасное получение позиции от LinuxCNC."""
        try:
            self._stat.poll()
            return self._stat.position
        except linuxcnc.error as detail:
            print(f"Ошибка LinuxCNC: {detail}")
            sys.exit(1)
    
    def get_xy(self) -> Tuple[float, float]:
        """Получает текущие координаты X и Y."""
        pos = self.get_position()
        return round(pos[0], 2), round(pos[1], 2)
    
    def get_z(self) -> float:
        """Получает текущую координату Z."""
        pos = self.get_position()
        return round(pos[2], 2)
    
    def send_mdi_message(self, msg: str) -> None:
        """Отправляет сообщение через MDI."""
        self._command.mode(linuxcnc.MODE_MDI)
        self._command.wait_complete()
        self._command.mdi(f"(MSG, {msg})")
    
    def run_program(self, filepath: str, on_complete_callback=None) -> None:
        """Запускает программу и ожидает завершения."""
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
        """Возвращает массив точек перегиба траектории в формате [(x1, y1), ...]."""
        vsa_rad = math.radians(self.vsa)
        vsa2_rad = math.radians(self.vsa2)

        # Точки перегиба
        y0 = self.y_offset
        x0 = -self.fd / 2
        x1 = -self.vsdtr / 2
        y1 = self.y_offset
        
        # Вторая точка перегиба
        x2 = -self.vsdtr / 2 + self.vsw * np.cos(vsa_rad)
        y2 = -self.vsw * math.sin(vsa_rad) + self.y_offset
        
        # Третья точка: максимальная глубина
        x3 = x2 + (self.vsd - self.vsw * math.sin(vsa_rad)) / math.tan(vsa2_rad)
        y3 = min(-self.vsd + self.y_offset, 0)

        return [(x0, y0), (x1, y1), (x2, y2), (x3, y3)]
#------------------------------------------------------------------------------------------------------------------------#
class HandlerClass:
    """Основной класс обработчика GUI для управления обработкой седел клапанов."""
    
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
        
        self.textview = self.builder.get_object("prog")
        if self.textview:
            self.textbuffer = self.textview.get_buffer()

    def file_path(self, file_name: str) -> str:
        """Возвращает полный путь к файлу в директории проекта."""
        name_pr = self.builder.get_object("name_project").get_text()
        file_path = os.path.join(HOME_DIR, name_pr, file_name)
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        return file_path

    def get_entry_value(self, entry_name, default=0.0):
        """Получает числовое значение из поля ввода GUI."""
        entry = self.builder.get_object(entry_name)
        if not entry:
            return default
        try:
            return float(entry.get_text().replace(',', '.'))
        except ValueError:
            return default

    def get_current_pilot(self):
        """Получает параметры для поиска центра отверстия."""
        return (
            self.get_entry_value("f_slow"),
            self.get_entry_value("f_fast"),
            self.get_entry_value("d_seat"),
            self.get_entry_value("h_probe_depth"),
            self.get_entry_value("probe_retract")
        )
    
    def _get_linuxcnc_position(self):
        """Безопасное получение позиции от LinuxCNC."""
        try:
            s = linuxcnc.stat()
            s.poll()
            return s.position
        except linuxcnc.error as detail:
            print(f"Ошибка LinuxCNC: {detail}")
            sys.exit(1)
    
    def get_xy(self):
        """Получает текущие координаты X и Y."""
        pos = self._get_linuxcnc_position()
        return round(pos[0], 2), round(pos[1], 2)
    
    def get_z(self):
        """Получает текущую координату Z."""
        pos = self._get_linuxcnc_position()
        return round(pos[2], 2)

    def create_program(self, mode: str = "pilot"):
        """Создать программу поиска центра.
        
        Args:
            mode: режим работы - "pilot" (поиск центра пилота) или "hole" (поиск центра отверстия)
        """
        # Получаем текущие значения параметров
        f_slow, f_fast, d_seat, h_probe_depth, probe_retract = self.get_current_pilot()
        
        if mode == "hole":
            # Программа поиска центра ОТВЕРСТИЯ
            start_offset = d_seat / 2 - 2  # Начинаем ближе к центру, на 2мм меньше радиуса
            probe_distance = d_seat - 4    # Расстояние зондирования чуть меньше диаметра
            
            programm = (
                f'(MSG, Поиск центра отверстия)\n'
                f'G54 G90\n'
                f'G21 G91\n'
                f'#5 = #<_x>\n'
                f'#6 = #<_y>\n'
                # Зондирование по X: левая стенка
                f'G0 X-{start_offset}\n'
                f'G0 Z{-h_probe_depth}\n'
                f'G38.2 X-{probe_distance} F{f_fast}\n'
                f'G38.4 X{probe_retract*1.2} F{f_slow}\n'
                f'G38.2 X-{probe_retract*3} F{f_slow}\n'
                f'#1 = #<_x>\n'
                f'G0 X{probe_retract*2}\n'
                f'G0 Z{h_probe_depth}\n'
                # Зондирование по X: правая стенка
                f'G90\n'
                f'G0 X#5\n'
                f'G91\n'
                f'G0 X{start_offset}\n'
                f'G0 Z{-h_probe_depth}\n'
                f'G38.2 X{probe_distance} F{f_fast}\n'
                f'G38.4 X{-probe_retract*1.2} F{f_slow}\n'
                f'G38.2 X{probe_retract*3} F{f_slow}\n'
                f'#2 = #<_x>\n'
                f'G0 X{-probe_retract*2}\n'
                f'G0 Z{h_probe_depth}\n'
                # Зондирование по Y: нижняя стенка
                f'G90\n'
                f'G0 X#5\n'
                f'G0 Y#6\n'
                f'G91\n'
                f'G0 Y-{start_offset}\n'
                f'G0 Z{-h_probe_depth}\n'
                f'G38.2 Y-{probe_distance} F{f_fast}\n'
                f'G38.4 Y{probe_retract*1.2} F{f_slow}\n'
                f'G38.2 Y-{probe_retract*3} F{f_slow}\n'
                f'#3 = #<_y>\n'
                f'G0 Y{probe_retract*2}\n'
                f'G0 Z{h_probe_depth}\n'
                # Зондирование по Y: верхняя стенка
                f'G90\n'
                f'G0 X#5\n'
                f'G0 Y#6\n'
                f'G91\n'
                f'G0 Y{start_offset}\n'
                f'G0 Z{-h_probe_depth}\n'
                f'G38.2 Y{probe_distance} F{f_fast}\n'
                f'G38.4 Y{-probe_retract*1.2} F{f_slow}\n'
                f'G38.2 Y{probe_retract*3} F{f_slow}\n'
                f'#4 = #<_y>\n'
                f'G0 Y{-probe_retract*2}\n'
                f'G0 Z{h_probe_depth}\n'
                # Вычисление центра
                f'G90\n'
                f'G0 X[ [#1 + #2] / 2 ] Y[ [#3 + #4] / 2 ]\n'
                f'M2')
        else:
            # Программа поиска центра ПИЛОТА (оригинальный код)
            programm = (
                f'G54 G90\n'
                f'G21 G91\n'
                f'#5 = #<_x>\n'
                f'#6 = #<_y>\n'
                f'G0 X{d_seat/2+2}\n'
                f'G0 Z{-h_probe_depth}\n'
                f'G38.2 X{-d_seat/2+2} F{f_fast}\n'
                f'G38.4 X{probe_retract*1.2} F{f_slow}\n'
                f'G38.2 X{-probe_retract*3} F{f_slow}\n'
                f'#1 = #<_x>\n'
                f'G0 X{probe_retract*2}\n'
                f'G0 Z{h_probe_depth}\n'
                f'G90\n'
                f'G0 X#5\n'
                f'G91\n'
                f'G0 X{-d_seat/2-2}\n'
                f'G0 Z{-h_probe_depth}\n'
                f'G38.2 X{d_seat/2+2} F{f_fast}\n'
                f'G38.4 X{-probe_retract*1.2} F{f_slow}\n'
                f'G38.2 X{probe_retract*3} F{f_slow}\n'
                f'#2 = #<_x>\n'
                f'G0 X{-probe_retract*2}\n'
                f'G0 Z{h_probe_depth}\n'
                f'G90\n'
                f'G0 X#5\n'
                f'G91\n'
                f'G0 Y{d_seat/2+2}\n'
                f'G0 Z{-h_probe_depth}\n'
                f'G38.2 Y{-d_seat/2+2} F{f_fast}\n'
                f'G38.4 Y{probe_retract*1.2} F{f_slow}\n'
                f'G38.2 Y{-probe_retract*3} F{f_slow}\n'
                f'#3 = #<_y>\n'
                f'G0 Y{probe_retract*2}\n'
                f'G0 Z{h_probe_depth}\n'
                f'G90\n'
                f'G0 Y#6\n'
                f'G91\n'
                f'G0 Y{-d_seat/2-2}\n'
                f'G0 Z{-h_probe_depth}\n'
                f'G38.2 Y{d_seat/2+2} F{f_fast}\n'
                f'G38.4 Y{-probe_retract*1.2} F{f_slow}\n'
                f'G38.2 Y{probe_retract*3} F{f_slow}\n'
                f'#4 = #<_y>\n'
                f'G0 Y{-probe_retract*2}\n'
                f'G0 Z{h_probe_depth}\n'
                f'G90\n'
                f'G0 Y#6\n'
                f'(MSG, Разверните центроискатель на 180 градусов )\n'
                f'G54 G90\n'
                f'G21 G91\n'
                f'#5 = #<_x>\n'
                f'#6 = #<_y>\n'
                f'G0 X{d_seat/2+2}\n'
                f'G0 Z{-h_probe_depth}\n'
                f'G38.2 X{-d_seat/2+2} F{f_fast}\n'
                f'G38.4 X{probe_retract*1.2} F{f_slow}\n'
                f'G38.2 X{-probe_retract*3} F{f_slow}\n'
                f'#7 = #<_x>\n'
                f'G0 X{probe_retract*2}\n'
                f'G0 Z{h_probe_depth}\n'
                f'G90\n'
                f'G0 X#5\n'
                f'G91\n'
                f'G0 X{-d_seat/2-2}\n'
                f'G0 Z{-h_probe_depth}\n'
                f'G38.2 X{d_seat/2+2} F{f_fast}\n'
                f'G38.4 X{-probe_retract*1.2} F{f_slow}\n'
                f'G38.2 X{probe_retract*3} F{f_slow}\n'
                f'#8 = #<_x>\n'
                f'G0 X{-probe_retract*2}\n'
                f'G0 Z{h_probe_depth}\n'
                f'G90\n'
                f'G0 X#5\n'
                f'G91\n'
                f'G0 Y{d_seat/2+2}\n'
                f'G0 Z{-h_probe_depth}\n'
                f'G38.2 Y{-d_seat/2+2} F{f_fast}\n'
                f'G38.4 Y{probe_retract*1.2} F{f_slow}\n'
                f'G38.2 Y{-probe_retract*3} F{f_slow}\n'
                f'#9 = #<_y>\n'
                f'G0 Y{probe_retract*2}\n'
                f'G0 Z{h_probe_depth}\n'
                f'G90\n'
                f'G0 Y#6\n'
                f'G91\n'
                f'G0 Y{-d_seat/2-2}\n'
                f'G0 Z{-h_probe_depth}\n'
                f'G38.2 Y{d_seat/2+2} F{f_fast}\n'
                f'G38.4 Y{-probe_retract*1.2} F{f_slow}\n'
                f'G38.2 Y{probe_retract*3} F{f_slow}\n'
                f'#10 = #<_y>\n'
                f'G0 Y{-probe_retract*2}\n'
                f'G0 Z{h_probe_depth}\n'
                f'G90\n'
                f'G0 Y#6\n'
                f'G0 X[ [#1 + #2 + #7 + #8] / 4 ] Y[ [#3 + #4 + #9 + #10] / 4 ]\n'
                f'M2')
        
        with open(self.file_path("centr_pr.ngc"), "w", encoding="utf-8") as f:
            f.write(programm)
        with open(self.file_path("centr_pr.ngc"), "w", encoding="utf-8") as f:
            f.write(programm)
        #-------------------------------------------------------------

    def find_center(self, widget):
        self.create_program()
        
        c.reset_interpreter()
        c.program_open(self.file_path("centr_pr.ngc")) 
        c.wait_complete(100)
        c.mode(linuxcnc.MODE_AUTO)
        c.auto(linuxcnc.AUTO_RUN, 0)


        # Ожидаем завершения программы
        while True:
            s.poll()
            if s.interp_state == linuxcnc.INTERP_IDLE:
                self.message_mdi("Программа завершена.")
                coord = self.get_xy()
                break
            c.wait_complete(10)

        coord = self.get_xy()

        self.x = coord[0]
        self.y = coord[1]
        coord_str = f"X{self.x} Y{self.y}"

        # Найти GtkLabel по ID
        label = self.builder.get_object("val_centre")
        if label:
            label.set_text(coord_str)
        else:
            self.message_mdi("Label 'val_centre' не найден!")

    def _update_table_ui(self):
        """Обновляет отображение таблицы в GUI."""
        grid = Gtk.Grid()
        grid.set_column_spacing(10)
        grid.set_row_spacing(5)

        for col, title in enumerate(TABLE_HEADERS):
            label = Gtk.Label(label=f"<b>{title}</b>", use_markup=True)
            grid.attach(label, col, 0, 1, 1)

        for row_index, row_data in enumerate(self.table_data, start=1):
            for col_index, cell in enumerate(row_data):
                label = Gtk.Label(label=cell)
                grid.attach(label, col_index, row_index, 1, 1)

        frame = self.builder.get_object("table_centers")
        for child in frame.get_children():
            frame.remove(child)

        frame.add(grid)
        frame.show_all()
        
        # Сохраняем в файл
        if self.table_data:
            pd.DataFrame(self.table_data).to_csv(
                self.file_path("table_center.csv"), index=False, header=False
            )

    def _get_valve_type(self):
        """Определяет тип клапана из радиокнопок."""
        valve_in_btn = self.builder.get_object("valv_in")
        valve_out_btn = self.builder.get_object("valv_out")
        
        if valve_in_btn and valve_in_btn.get_active():
            return "впускной"
        elif valve_out_btn and valve_out_btn.get_active():
            return "выпускной"
        return "неизвестно"

    def _clear_input_fields(self):
        """Очищает поля ввода после добавления записи."""
        for field_name in ["val_num", "curva_koef", "comment"]:
            entry = self.builder.get_object(field_name)
            if entry:
                entry.set_text("0.0" if field_name == "curva_koef" else "")

    def _build_table_row(self, val_num_tab):
        """Создаёт строку данных для таблицы."""
        coord = self.get_xy()
        return [
            val_num_tab,
            f"{coord[0]:.2f}",
            f"{coord[1]:.2f}",
            self._get_valve_type(),
            self.get_entry_value("curva_koef"),
            self.builder.get_object("comment").get_text()
        ]

    def change_table(self, widget):
        """Изменяет или добавляет запись в таблице центров."""
        val_num = self.get_entry_value("val_num")
        val_num_tab = int(val_num) if val_num != 0 else len(self.table_data) + 1
        
        row = self._build_table_row(val_num_tab)
        
        # Ищем существующую запись с таким номером
        for i, existing_row in enumerate(self.table_data):
            if int(existing_row[0]) == val_num_tab:
                self.table_data[i] = row
                break
        else:
            self.table_data.append(row)

        self._clear_input_fields()
        self._update_table_ui()

    def add_table(self, widget):
        """Добавляет новую запись в таблицу центров."""
        val_num = self.get_entry_value("val_num")
        val_num_tab = int(val_num) if val_num != 0 else len(self.table_data) + 1
        
        row = self._build_table_row(val_num_tab)
        
        # Ищем существующую запись с таким номером
        for i, existing_row in enumerate(self.table_data):
            if int(existing_row[0]) == val_num_tab:
                self.table_data[i] = row
                break
        else:
            self.table_data.append(row)

        self._clear_input_fields()
        self._update_table_ui()

    def remove_row(self, widget):
        """Удаляет последнюю строку из таблицы."""
        if self.table_data:
            self.table_data.pop()
        self._update_table_ui()

    def load_table(self, wiget):
        f_name  = self.builder.get_object("file_table")
        filename = f_name.get_filename() # получаем путь и имя файла

        filename = filename if filename is not None else self.file_path("table_center.csv") # если имя файла не задано ставим имя по умолчанию

        data = pd.read_csv(filename, header=None) #читаем сохраненный файл

        data_load = data.astype(str).values.tolist()
        self.table_data = data_load # перезаписываем данные

        # Создаём GtkGrid для таблицы
        grid = Gtk.Grid()
        grid.set_column_spacing(10)
        grid.set_row_spacing(5)

        # Добавляем заголовки
        headers = ["№","X", "Y", "Тип", "Коэфф. кривизны","Примечание"]
        for col, title in enumerate(headers):
            label = Gtk.Label(label=f"<b>{title}</b>", use_markup=True)
            grid.attach(label, col, 0, 1, 1)

        # Добавляем строки из dataset
        for row_index, row_data in enumerate(self.table_data, start=1):
            for col_index, cell in enumerate(row_data):
                label = Gtk.Label(label=cell)
                grid.attach(label, col_index, row_index, 1, 1)

        # Найти GtkFrame и очистить от старых дочерних виджетов
        frame = self.builder.get_object("table_centers")
        for child in frame.get_children():
            frame.remove(child)

        # Вставить новую таблицу
        frame.add(grid)
        frame.show_all()

        

#--------------------------------------------------------------------------------------------------------------#
    def get_current_values(self):
        f       = self.get_entry_value("EntryF")
        rpm     = self.get_entry_value("EntryRPM")
        fpp     = self.get_entry_value("EntryFPP")
        return f, rpm, fpp

    def get_current_values_in(self):
        
        fd_in      = self.get_entry_value("EntryFD_in")
        vsd_in     = self.get_entry_value("EntryVSD_in")
        vsdtr_1_in = self.get_entry_value("EntryVSDtr1_in")
        vsdtr_2_in = self.get_entry_value("EntryVSDtr2_in")
        vsa_1_in   = self.get_entry_value("EntryVSA1_in")
        vsa_2_in   = self.get_entry_value("EntryVSA2_in")
        vsw_1_in   = self.get_entry_value("EntryVSW1_in")
        vsw_2_in   = self.get_entry_value("EntryVSW2_in")
        vsa2_1_in  = self.get_entry_value("EntryVSA21_in")
        vsa2_2_in  = self.get_entry_value("EntryVSA22_in")
        ff_in      = self.get_entry_value("EntryFF_in")
        md_in      = self.get_entry_value("EntryMD_in")
        
        return fd_in, vsd_in, vsdtr_1_in, vsdtr_2_in, vsa_1_in, vsa_2_in, vsw_1_in, vsw_2_in, vsa2_1_in, vsa2_2_in, ff_in, md_in
    
    def get_current_values_out(self):
        fd_out      = self.get_entry_value("EntryFD_out")
        vsd_out     = self.get_entry_value("EntryVSD_out")
        vsdtr_1_out = self.get_entry_value("EntryVSDtr1_out")
        vsdtr_2_out = self.get_entry_value("EntryVSDtr2_out")
        vsa_1_out   = self.get_entry_value("EntryVSA1_out")
        vsa_2_out   = self.get_entry_value("EntryVSA2_out")
        vsw_1_out   = self.get_entry_value("EntryVSW1_out")
        vsw_2_out   = self.get_entry_value("EntryVSW2_out")
        vsa2_1_out  = self.get_entry_value("EntryVSA21_out")
        vsa2_2_out  = self.get_entry_value("EntryVSA22_out")
        ff_out      = self.get_entry_value("EntryFF_out")
        md_out      = self.get_entry_value("EntryMD_out")
        return fd_out, vsd_out, vsdtr_1_out, vsdtr_2_out, vsa_1_out, vsa_2_out, vsw_1_out, vsw_2_out, vsa2_1_out, vsa2_2_out, ff_out, md_out

    def create_valve_objects(self):
        _, _, fpp = self.get_current_values()
        fd_in, vsd_in, vsdtr_1_in, vsdtr_2_in, vsa_1_in, vsa_2_in, vsw_1_in, vsw_2_in, vsa2_1_in, vsa2_2_in, ff_in, md_in = self.get_current_values_in()
        fd_out, vsd_out, vsdtr_1_out, vsdtr_2_out, vsa_1_out, vsa_2_out, vsw_1_out, vsw_2_out, vsa2_1_out, vsa2_2_out, ff_out, md_out = self.get_current_values_out()

        blank_valve_in = ValveSeatContour(fd=fd_in, vsdtr=vsdtr_1_in, vsa=vsa_1_in, vsw=vsw_1_in, vsa2=vsa2_1_in, vsd=vsd_in, y_offset=0, md=md_in)
        valve_in = ValveSeatContour(fd=fd_in, vsdtr=vsdtr_2_in, vsa=vsa_2_in, vsw=vsw_2_in, vsa2=vsa2_2_in, vsd=vsd_in, y_offset=-ff_in, md=md_in)

        y_offsets_in = np.arange(-ff_in, vsd_in, fpp)
        valve_processing_in = [ 
            ValveSeatContour(fd=fd_in, vsdtr=vsdtr_2_in, vsa=vsa_2_in, vsw=vsw_2_in, vsa2=vsa2_2_in, vsd=vsd_in, y_offset=y, md=md_in) 
            for y in y_offsets_in ]

        blank_valve_out = ValveSeatContour(fd=fd_out, vsdtr=vsdtr_1_out, vsa=vsa_1_out, vsw=vsw_1_out, vsa2=vsa2_1_out, vsd=vsd_out, y_offset=0, md=md_out)
        valve_out = ValveSeatContour(fd=fd_out, vsdtr=vsdtr_2_out, vsa=vsa_2_out, vsw=vsw_2_out, vsa2=vsa2_2_out, vsd=vsd_out, y_offset=-ff_out, md=md_out)

        y_offsets = np.arange(-ff_out, vsd_out, fpp)
        valve_processing_out = [
            ValveSeatContour(fd=fd_out, vsdtr=vsdtr_2_out, vsa=vsa_2_out, vsw=vsw_2_out, vsa2=vsa2_2_out, vsd=vsd_out, y_offset=y, md=md_out)
            for y in y_offsets
        ]

        return blank_valve_in, valve_in, valve_processing_in, md_in, blank_valve_out, valve_out, valve_processing_out, md_out
    
    def check_polylines_intersect(self, bv_breakpoints, proc_valve):
        """
        Проверяет, пересекаются ли две ломаные линии.
        
        Аргументы:
        - bv_breakpoints: список кортежей (x, y)
        - proc_valve: список кортежей (x, y)
        
        Возвращает:
        - True, если есть пересечение
        - False, если пересечений нет
        """
        # Преобразуем в линии Shapely
        line1 = LineString(bv_breakpoints)
        line2 = LineString(proc_valve)
        
        return line1.intersects(line2)

    def check_polylines_interpolate(self, bv_breakpoints, proc_valve):
        """
        Проверяет, находится ли одна ломаная выше другой.
        
        Аргументы:
        - bv_breakpoints: список кортежей (x, y)
        - proc_valve: список кортежей (x, y)
        
        Возвращает:
        - True, если выше
        - False, если нет
        """
        # Преобразуем в линии Shapely
        line1 = LineString(bv_breakpoints)
        line2 = LineString(proc_valve)
        
        return all(p2[1] > line1.interpolate(line1.project(Point(p2))).y for p2 in line2.coords)
    
    def valve_proc (self, valve_processing, blank_valve, md):
        clean_proc = []
        bv_breakpoints_b = blank_valve.get_breakpoints()
        for proc_valve in valve_processing:
        
            break_x_proc, break_y_proc = zip(*proc_valve.get_breakpoints())
            break_x, break_y = list(break_x_proc), list(break_y_proc)

            # Обрезаем по внутреннему диаметру заготовки
            if break_x[3] >= -md/2:
                break_y[3] = break_y[2] + (-md/2 - break_x[2]) * (break_y[3] - break_y[2]) / (break_x[3] - break_x[2])
                break_x[3] = -md/2
            else:
                break_x.append(-md/2)
                break_y.append(break_y[3])
            
            # Проверка на пересечение с контуром обработки
            above_up = self.check_polylines_interpolate(bv_breakpoints_b, list(zip(break_x, break_y)))
            above = self.check_polylines_intersect(bv_breakpoints_b, list(zip(break_x, break_y)))

            if (not above)&above_up:
                break

            # Обрезаем траектории по Z =0
            for i in range(len(break_y_proc)):
                if break_y_proc[i] > 0:
                    if break_y[i+1] - break_y[i] !=0:
                        break_x[i] = break_x[i] + (0 - break_y[i]) * (break_x[i+1] - break_x[i]) / (break_y[i+1] - break_y[i])
                    break_y[i] = 0

            # Убираем лишние проходы по Z=0
            for i in range(len(break_y)-1, 0, -1):
                if break_y[i] == 0 and break_y[i-1] == 0:
                    break_x[i-1] = break_x[i]

        
            clean_proc.append(list(zip(break_x, break_y)))

        reversed_clean_proc = [s[::-1] for s in clean_proc[::-1]]
        return reversed_clean_proc

#-------------------------------------------------------------------------------------------------------------------#
    def prog_valve_seat(self, clean_proc: list[list[tuple[float, float]]], md: float, start: int = 0) -> str:
        """
        Генерирует G-код программу для обработки седла клапана на станке с ЧПУ.
        
        Функция создает последовательность команд G-кода для обработки профилей 
        седла клапана, начиная с указанного прохода. Каждый проход состоит из 
        последовательности точек (координат), которые обрабатываются инструментом.
        
        Параметры:
        -----------
        clean_proc : list[list[tuple[float, float]]]
            Список проходов обработки, где каждый проход представляет собой 
            список точек. Каждая точка - это кортеж (U, Z) координат.
            Формат: [[(U1,Z1), (U2,Z2), ...], [(U1,Z1), (U2,Z2), ...], ...]
            где U - радиальная координата, Z - осевая координата
        
        start : int, optional
            Номер прохода, с которого начинается обработка (начиная с 0).
            По умолчанию 0 (обработка со всех проходов)
        
        Возвращает:
        -----------
        str
            Полная программа G-кода в виде строки, включающая:
            - Преамбулу с настройками станка
            - Основную программу перемещений
            - Завершающую часть с возвратом и остановкой
        
        Пример структуры clean_proc:
        --------------------------
        clean_proc = [
            [(10.5, 0.0), (10.5, -2.0), (8.0, -2.0)],  # Проход 0
            [(10.0, 0.0), (10.0, -1.5), (7.5, -1.5)],  # Проход 1
            [(9.5, 0.0), (9.5, -1.0), (7.0, -1.0)]     # Проход 2
        ]
        
        Логика работы:
        --------------
        1. Проверяет корректность начального индекса прохода
        2. Получает текущие параметры обработки (подача, обороты, подача на проход)
        3. Формирует преамбулу G-кода с настройками станка
        4. Генерирует основную программу обработки:
        - Перебирает проходы начиная с указанного индекса
        - Для каждой точки в проходе генерирует команду G1 (линейная интерполяция)
        - Исключает дублирующиеся последовательные команды
        5. Добавляет завершающую часть программы:
        - Обработка кромки в конечной точке
        - Возврат в начальную позицию
        - Остановка программы
        
        G-код команды:
        --------------
        G21  - Установка миллиметров как единиц измерения
        G90  - Абсолютная система координат
        M3 S{rpm} - Включение шпинделя с заданными оборотами
        G1 U{U} Z{Z} F{f} - Линейная интерполяция с подачей
        G0 U{U} Z{Z} - Быстрое перемещение
        M30  - Конец программы и возврат в начало
        
        Обработка дубликатов:
        --------------------
        Функция автоматически исключает последовательные одинаковые команды,
        чтобы оптимизировать программу и избежать лишних остановок.
        
        Raises:
        -------
        IndexError
            Может возникнуть при некорректной структуре данных clean_proc
        TypeError  
            При передаче неверного типа данных в параметрах
        """
        
        if start > len(clean_proc):
            self.message_mdi('Не корректный номер прохода для начала')
            return ''
        
        programm = ''
        string = ''
        pre_string = ''
        f, _, _ = self.get_current_values()

        for i, str_item in enumerate(clean_proc[start:len(clean_proc)], start=start):
            for j, point in enumerate(str_item):
                if j ==0:
                    string = f'G0 U{point[0]:.2f} Z{point[1]:.2f} F{f} ; Проход {i} \n '
                else:
                    string = f'G1 U{point[0]:.2f} Z{point[1]:.2f} F{f} ; Проход {i} \n '

                if pre_string != string:
                    programm = programm + string
                    pre_string = string
            if i != len(clean_proc)-1:
                programm = programm + f'G0 U{-md/2} Z{point[1]:.2f} F{f} ; Конец прохода {i} \n '
        end_x = clean_proc[-1][-1][0]
        start_x = clean_proc[0][0][0]
        
        programm = programm + f"G1 U{end_x:.3f} Z0  ; Обрабатываем кромку \n"
        programm = programm + f"G0 U{start_x:.3f} Z0  ; Возвращаемся в исходное положение \n"
        
        
        return programm
    
    def valve_gcode_suffix(self):
         return f"""
G53 G0 Z{20+self.z0_in} ; Поднимаем Z
M30  ; Конец программы
"""
    
    def valve_gcode_prefix(self):
        _, rpm, _ = self.get_current_values() 
        return f"""
G21  ; Используем миллиметры
G90  ; Абсолютные координаты
M3 S{rpm} ; Старт шпинделя
"""
    
    def z_null(self):
        return f"""
G10 L20 P0 Z0 ; Устанавливаем новый Z0
"""

#-------------------------------------------------------------------------------------------------------------------#

    def _draw_valve_seat_common(self, container_name, blank_valve, valve, valve_processing, md):
        """Общая функция для отрисовки контуров клапана"""
        container = self.builder.get_object(container_name)

        # Полностью очищаем контейнер
        for child in container.get_children():
            container.remove(child)

        # Создаем фигуру
        fig = Figure(figsize=(9, 9), dpi=100)
        ax = fig.add_subplot(111)

        # Отрисовка контура заготовки
        bv_breakpoints_b = blank_valve.get_breakpoints()
        x_b, y_b = zip(*bv_breakpoints_b)
        ax.plot(x_b, y_b, label="Контур заготовки", color="blue")

        # Отрисовка контура клапана
        bv_breakpoints = valve.get_breakpoints()
        x_v, y_v = zip(*bv_breakpoints)
        ax.plot(x_v, y_v, label="Контур клапана", color="orange")

        # Отрисовка обработки
        clean_proc = self.valve_proc(valve_processing, blank_valve, md)
        for i in range(len(clean_proc)):
            x, y = zip(*clean_proc[i])
            ax.scatter(x, y, color="green", zorder=1, s=5)
            ax.plot(x, y, color="green", lw=0.5, linestyle="--")

        # Настройка графика
        ax.axhline(0, color="gray", linestyle="--", label="Z0")
        ax.axvline(-md/2, color="brown", linestyle="--", label="Внутренний диаметр заготовки")
        ax.legend()
        ax.set_xlabel("U")
        ax.set_ylabel("Z")
        ax.set_aspect('equal', adjustable='datalim')
        ax.axis("equal")
        ax.grid()

        # Создаем canvas и обновляем GUI
        self.canvas = FigureCanvas(fig)
        container.add(self.canvas)
        container.show_all()

    def draw_valve_seat_in(self):
        """Отрисовка внутреннего контура клапана"""
        blank_valve, valve, valve_processing_in, md, _, _, _, _ = self.create_valve_objects()
        self._draw_valve_seat_common("proc_kontur_box", blank_valve, valve, valve_processing_in, md)

    def draw_valve_seat_out(self):
        """Отрисовка внешнего контура клапана"""
        _, _, _, _, blank_valve, valve, valve_processing_out, md = self.create_valve_objects()
        self._draw_valve_seat_common("proc_kontur_box2", blank_valve, valve, valve_processing_out, md)


    
#-------------------------------------------------------------------------------------------------------------------#



    def on_draw_valve_seat_clicked(self, widget):
        self.nhits += 1
        hits_label = self.builder.get_object('hits')
        if hits_label:
            hits_label.set_label("Hits: %d" % (self.nhits))
        _, _, fpp = self.get_current_values()
        
        container = self.builder.get_object("proc_kontur_box")

        if not container:
            self.log_message("Контейнер 'proc_kontur_box' не найден")
            return  # Выход, если контейнер не найден

        if self.canvas:
            container.remove(self.canvas) 
            self.canvas = None

        self.draw_valve_seat_in()
        self.draw_valve_seat_out()

    def save_valves(self, widget):
        # Сохраняем параметра в файл values.json в папку проекта
        f, rpm, fpp = self.get_current_values()
        fd_in, vsd_in, vsdtr_1_in, vsdtr_2_in, vsa_1_in, vsa_2_in, vsw_1_in, vsw_2_in, vsa2_1_in, vsa2_2_in, ff_in, md_in = self.get_current_values_in()
        fd_out, vsd_out, vsdtr_1_out, vsdtr_2_out, vsa_1_out, vsa_2_out, vsw_1_out, vsw_2_out, vsa2_1_out, vsa2_2_out, ff_out, md_out = self.get_current_values_out()

        # Сохраняем в файл
        data = {
            "f": f,
            "rpm": rpm,
            "fpp": fpp,
            "fd_in": fd_in,
            "vsd_in": vsd_in, 
            "vsdtr_1_in": vsdtr_1_in,
            "vsdtr_2_in": vsdtr_2_in,
            "vsa_1_in": vsa_1_in,
            "vsa_2_in": vsa_2_in,
            "vsw_1_in": vsw_1_in,
            "vsw_2_in": vsw_2_in,
            "vsa2_1_in": vsa2_1_in,
            "vsa2_2_in": vsa2_2_in, 
            "ff_in": ff_in, 
            "md_in": md_in,
            "fd_out": fd_out, 
            "vsd_out": vsd_out, 
            "vsdtr_1_out": vsdtr_1_out, 
            "vsdtr_2_out": vsdtr_2_out, 
            "vsa_1_out": vsa_1_out, 
            "vsa_2_out": vsa_2_out, 
            "vsw_1_out": vsw_1_out, 
            "vsw_2_out": vsw_2_out, 
            "vsa2_1_out": vsa2_1_out, 
            "vsa2_2_out": vsa2_2_out, 
            "ff_out": ff_out, 
            "md_out": md_out
        }

        try:
            with open(self.file_path("values.json"), "w", encoding="utf-8") as file:
                json.dump(data, file, indent=4, ensure_ascii=False)
            self.message_mdi("Значения сохранены в values.json")
        except Exception as e:
            self.message_mdi(f"Ошибка при сохранении в файл: {e}")


    def load_values(self, widget):
        f_name  = self.builder.get_object("file_data")
        filename = f_name.get_filename() # получаем путь и имя файла
        filename = filename if filename is not None else self.file_path("values.json") # если имя файла не задано ставим имя по умолчанию

        values = [
            'f', 'rpm', 'fpp', 
            'fd_in', 'vsd_in', 'vsdtr_1_in', 'vsdtr_2_in', 'vsa_1_in', 'vsa_2_in', 'vsw_1_in', 'vsw_2_in', 'vsa2_1_in', 'vsa2_2_in', 'ff_in', 'md_in', 
            'fd_out', 'vsd_out', 'vsdtr_1_out', 'vsdtr_2_out', 'vsa_1_out', 'vsa_2_out', 'vsw_1_out', 'vsw_2_out', 'vsa2_1_out', 'vsa2_2_out', 'ff_out', 'md_out'
        ]
        
        name_id = [
            "EntryF", "EntryRPM", "EntryFPP",
            "EntryFD_in", "EntryVSD_in", "EntryVSDtr1_in", "EntryVSDtr2_in", "EntryVSA1_in", "EntryVSA2_in", "EntryVSW1_in", "EntryVSW2_in", "EntryVSA21_in", 
            "EntryVSA22_in", "EntryFF_in", "EntryMD_in",
            "EntryFD_out", "EntryVSD_out", "EntryVSDtr1_out", "EntryVSDtr2_out", "EntryVSA1_out", "EntryVSA2_out", "EntryVSW1_out", "EntryVSW2_out", "EntryVSA21_out",
            "EntryVSA22_out", "EntryFF_out", "EntryMD_out"
        ]


        try:
            with open(filename, "r", encoding="utf-8") as file:
                data = json.load(file)
            # Восстанавливаем значения
            for val, ent in zip(values, name_id):
                value = data.get(val)
                entry = self.builder.get_object(ent)
                entry.set_text(str(value))
            
        except FileNotFoundError:
            self.message_mdi("Файл values.json не найден.")
            return (None,) * 27
        except Exception as e:
            self.message_mdi(f"Ошибка при загрузке из файла: {e}")
            return (None,) * 27
        
    def save_one_valve_prog(self, widget):
        '''
        Формирует файл с программой в G-кодах для обработки заданного седла с координатами из таблицы центров
        и с началом с заданного прохода.
        - номер седла берется из self.get_entry_value("valve number")
        - номер прохода self.get_entry_value("start pass"
        '''
        valve_number  = self.get_entry_value("valve number")
        start_pass = int(self.get_entry_value("start pass"))

        # Получаем список с центрами и типами седел
        table = self.table_data 
        if table == []:
            self.message_mdi("Таблица с координатами центров отсутствует")
        table_df = pd.DataFrame(table, columns=["№","X", "Y", "Тип", "Коэфф. кривизны","Примечание"])
        table_df["№"] = table_df["№"].astype(int)
        valve = table_df[table_df["№"] == valve_number]
        # задаем параметры в зависимости от центра седла
        if valve["Тип"].iloc[0] == "впускной":
            blank_valve, _, valve_processing, md, _, _, _, _ = self.create_valve_objects()
            Z0 = self.z0_in
        if valve["Тип"].iloc[0] == "выпускной":
            _, _, _, _, blank_valve, _, valve_processing, md = self.create_valve_objects()
            Z0 = self.z0_out

        programm = f'G90  ; Абсолютные координаты\n '
        programm = programm +f'G53 G0 Z{20+Z0+float(valve["Коэфф. кривизны"])}  ; Корректировка Z с учетом смещения и коэф. кривизны \n'
        programm = programm +f'G10 L20 P0 Z20 ; Задаем откорректированное значение Z20\n '
        programm = programm +f'G0 U{-md/2}; Переводим U на U{-md/2} \n'
        programm = programm +f'G53 G0 X{valve["X"].iloc[0]} Y{valve["Y"].iloc[0]}; Переходим к клапану №{valve["№"].iloc[0]} \n'
        programm = programm +'M0 ; Пауза \n'

        clean_proc = self.valve_proc (valve_processing, blank_valve, md)
        programm = programm + self.prog_valve_seat(clean_proc, md, start_pass)

        programm = programm +f'G90  ; Абсолютные координаты\n '
        #programm = programm + f'G0 Z-{Z0+float(valve["Коэфф. кривизны"])} ; Возвращение к значению до корректировки\n '
        programm = programm + f'G10 L20 P0 Z0 ; Обнуление Z\n '

        prefix = self.valve_gcode_prefix()
        suffix = self.valve_gcode_suffix()

        # Сохраняем программу
        with open(self.file_path((f"valve_{valve_number}_start_pass{start_pass}.ngc")), "w", encoding="utf-8") as f:
            f.write(prefix + programm + suffix)



    def run_and_save_in(self, widget):
        start_pass_in = self.get_entry_value("EntryFD_start_pass_in")
        blank_valve, _, valve_processing, md, _, _, _, _ = self.create_valve_objects()
        clean_proc = self.valve_proc (valve_processing, blank_valve, md)
        prefix = self.valve_gcode_prefix()
        suffix = self.valve_gcode_suffix()
        programm = self.prog_valve_seat(clean_proc, md, int(start_pass_in))
        
        # Сохраняем программу
        with open(self.file_path("valve_in.ngc"), "w", encoding="utf-8") as f:
            f.write(prefix + programm + suffix)

        # Выполняем программу 
        c.reset_interpreter()
        c.program_open(self.file_path("valve_in.ngc")) 
        c.mode(linuxcnc.MODE_AUTO)
        c.auto(linuxcnc.AUTO_RUN, 0)

        # Ожидаем завершения программы
        while True:
            s.poll()
            if s.interp_state == linuxcnc.INTERP_IDLE:
                self.message_mdi("Программа завершена.")
                break
            c.wait_complete(10)
        

    def run_and_save_out(self, widget):
        start_pass_out = self.get_entry_value("EntryFD_start_pass_out")
        _, _, _, _, blank_valve, _, valve_processing, md = self.create_valve_objects()
        clean_proc = self.valve_proc (valve_processing, blank_valve, md)
        prefix = self.valve_gcode_prefix()
        suffix = self.valve_gcode_suffix()
        programm = self.prog_valve_seat(clean_proc, md, int(start_pass_out))
        # Сохраняем программу
        with open(self.file_path("valve_out.ngc"), "w", encoding="utf-8") as f:
            f.write(prefix + programm + suffix)

        # Выполняем программу 
        c.reset_interpreter()
        c.program_open(self.file_path("valve_out.ngc")) 
        c.mode(linuxcnc.MODE_AUTO)
        c.auto(linuxcnc.AUTO_RUN, 0)

        # Ожидаем завершения программы
        while True:
            s.poll()
            if s.interp_state == linuxcnc.INTERP_IDLE:
                self.message_mdi("Программа завершена.")
                break
            c.wait_complete(10)

    def message_mdi(self, msg: str):
        c.mode(linuxcnc.MODE_MDI)
        c.wait_complete() # wait until mode switch executed
        c.mdi(f"(MSG, {msg})")

    def parse_to_set(self, input_str)->set:
        """
        Преобразует строку вида "1-4" или "1, 2, 3, 4" в множество целых чисел
        """
        result = set()
        
        # Убираем пробелы и разбиваем по запятым
        parts = input_str.replace(' ', '').split(',')
        
        for part in parts:
            try:
                if '-' in part:
                    # Обрабатываем диапазон
                    start, end = map(int, part.split('-'))
                    if start <= end:
                        result.update(range(start, end + 1))
                    else:
                        self.message_mdi(f"Предупреждение: Неверный диапазон {part}. Начало больше конца.")
                else:
                    # Обрабатываем отдельное число
                    result.add(int(part))
            except ValueError as e:
                self.message_mdi(f"Предупреждение: Не удалось преобразовать '{part}' в целое число. Пропущено.")
            except Exception as e:
                self.message_mdi(f"Предупреждение: Ошибка при обработке '{part}': {e}")
        
        return result
    
    def set_z0_in(self, widget):

        self.z0_in = self.get_z()
        coord_str = f"Z0 G53 {self.z0_in}"

        # Найти GtkLabel по ID
        label = self.builder.get_object("z0_in_G53")
        if label:
            label.set_text(coord_str)
        else:
            self.message_mdi("Label 'z0_in_G53' не найден!")

    def set_z0_out(self, widget):

        self.z0_out = self.get_z()
        coord_str = f"Z0 G53 {self.z0_out}"

        # Найти GtkLabel по ID
        label = self.builder.get_object("z0_out_G53")
        if label:
            label.set_text(coord_str)
        else:
            self.message_mdi("Label 'z0_out_G53' не найден!")


    def full_programm(self, widget):

        full_programm = 'M0 ; Пауза \n'
        Z0_in  = self.z0_in
        Z0_out = self.z0_out
        md_in  = self.get_entry_value("EntryMD_in")
        md_out = self.get_entry_value("EntryMD_out")
        start_pass_in = int(self.get_entry_value("start_pass_in"))
        start_pass_out = int(self.get_entry_value("start_pass_out"))

        # Получаем список с центрами и типами седел
        table = self.table_data 
        if table == []:
            self.message_mdi("Таблица с координатами центров отсутствует")

        # Получаем список седел которые будем обрабатывать     
        entry_numer_obj = self.builder.get_object("valve numbers")
        valves_numer = self.parse_to_set(entry_numer_obj.get_text())
        prefix = self.valve_gcode_prefix()
        suffix = self.valve_gcode_suffix()
        self.message_mdi(f'Обрабатываем седла {valves_numer}\n Для старта обработки \n снимите с паузы')
        #print(valves_numer)

    
        blank_valve_in, _, valve_processing_in, _, _, _, _, _ = self.create_valve_objects()
        clean_proc_in = self.valve_proc (valve_processing_in, blank_valve_in, md_in)
        programm_in = self.prog_valve_seat(clean_proc_in, md_in, start_pass_in)
        full_programm = full_programm + f'o101 sub\n'
        full_programm = full_programm + programm_in
        full_programm = full_programm + f'o101 endsub\n'

        _, _, _, _, blank_valve_out, _, valve_processing_out, _ = self.create_valve_objects()
        clean_proc_out = self.valve_proc (valve_processing_out, blank_valve_out, md_out)
        programm_out = self.prog_valve_seat(clean_proc_out, md_out, start_pass_out)
        full_programm = full_programm + f'o102 sub\n'
        full_programm = full_programm + programm_out
        full_programm = full_programm + f'o102 endsub\n'

        for row_index, row_data in enumerate(table, start=1):
            if int(row_data[0]) in valves_numer:
                if row_data[3] == "впускной":
                    full_programm = full_programm + f'G0 Z20 ;Обработка седла клапана {row_data[0]} {row_data[3]}\n '
                    full_programm = full_programm + f'G0 U{-md_in/2} ; Перемещение U на внутренний диаметр\n '
                    full_programm = full_programm + f'G53 G0 X{row_data[1]} Y{row_data[2]} ;Перемещение к центру седла клапана {row_data[0]}\n '
                    # full_programm = full_programm + f'G90  ; Абсолютные координаты\n '
                    full_programm = full_programm + f'G53 G0 Z{Z0_in+float(row_data[4])}  ; Корректировка Z с учетом смещения и коэф. кривизны \n'
                    full_programm = full_programm + f'G10 L20 P0 Z0 ; Обнуление Z\n '
                    full_programm = full_programm + f'o101 call ; Вызов подпрограммы обработки впускного седла\n '
                    # full_programm = full_programm + f'G0 Z-{Z0_in+float(row_data[4])} ; Возвращение к значению до корректировки\n '
                    # full_programm = full_programm + f'G10 L20 P0 Z0 ; Обнуление Z\n '
                if row_data[3] == "выпускной":
                    full_programm = full_programm + f'G0 Z20 ;Обработка седла клапана {row_data[0]} {row_data[3]}\n '
                    full_programm = full_programm + f'G0 U{-md_out/2} ; Перемещение U на внутренний диаметр\n '
                    full_programm = full_programm + f'G53 G0 X{row_data[1]} Y{row_data[2]} ;Перемещение к центру седла клапана {row_data[0]}\n '
                    # full_programm = full_programm + f'G90  ; Абсолютные координаты\n '
                    full_programm = full_programm + f'G53 G0 Z{Z0_out+float(row_data[4])} ; Корректировка Z с учетом смещения и коэф. кривизны \n'
                    full_programm = full_programm + f'G10 L20 P0 Z0 ; Обнуление Z\n '
                    full_programm = full_programm + f'o102 call ; Вызов подпрограммы обработки выпускного седла\n '
                    # full_programm = full_programm + f'G0 Z-{Z0_out+float(row_data[4])} ; Возвращение к значению до корректировки\n '
                    # full_programm = full_programm + f'G10 L20 P0 Z0 ; Обнуление Z\n '

        #print(prefix+full_programm+suffix)

        # Сохраняем программу
        full_pr_path = self.file_path("full_programm.ngc")
        print(full_pr_path)                              
        with open(self.file_path("full_programm.ngc"), "w", encoding="utf-8") as f:
            f.write(prefix+full_programm+suffix)

        # Выполняем программу 
        c.reset_interpreter()
        c.program_open(self.file_path("full_programm.ngc")) 
        c.mode(linuxcnc.MODE_AUTO)
        c.auto(linuxcnc.AUTO_RUN, 0)

        # Ожидаем завершения программы
        while True:
            s.poll()
            if s.interp_state == linuxcnc.INTERP_IDLE:
                print("Программа завершена.")
                break
            c.wait_complete(10)

#-------------------------------------------------------------------------------------------------------------------#
def get_handlers(halcomp, builder, useropts):
    return [HandlerClass(halcomp, builder, useropts)]