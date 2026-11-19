import sys
import time
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QTextEdit, QLabel)
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import QTimer, Qt
import pyqtgraph as pg
from pymodbus.client import ModbusSerialClient

DUMMY_MODE = False


class ValveTestBench(QMainWindow):
    def __init__(self):
        super().__init__()
        self.modbus = None
        self.setWindowTitle("Стенд тестирования перепускных клапанов")
        # Размер окна приближен к оригинальному (1179 x 487)
        self.resize(1180, 500)

        # --- Аппаратные настройки ---
        self.ADDR_DO = 1
        self.ADDR_AO = 2
        self.ADDR_FLOW = 86

        self.DO_VALVE_FWD_IN = 1
        self.DO_VALVE_REV_IN = 4
        self.DO_VALVE_FWD_OUT = 3
        self.DO_VALVE_REV_OUT = 2
        self.DO_PUMP_START = 5

        # --- Переменные состояния ---
        self.state = "IDLE"
        self.start_time = 0
        self.time_data = []
        self.pressure_data = []

        self.current_pressure = 0.0
        self.current_flow = 0
        self.fixed_open = 0.0

        self.init_ui()
        self.init_modbus()

        self.timer = QTimer()
        self.timer.timeout.connect(self.main_loop)
        self.timer.start(200)

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # ================= ЛЕВАЯ ПАНЕЛЬ (МНЕМОСХЕМА) =================
        left_pane = QWidget()
        left_pane.setFixedWidth(520)  # Фиксируем ширину под размер оригинальной схемы
        left_layout = QVBoxLayout(left_pane)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        self.schematic_label = QLabel()
        try:
            # Оригинальный размер PictureBox1 был 503 x 397
            pixmap = QPixmap("shema.png").scaled(503, 397, Qt.AspectRatioMode.IgnoreAspectRatio)
            self.schematic_label.setPixmap(pixmap)
            self.schematic_label.setFixedSize(503, 397)
        except Exception as e:
            self.schematic_label.setText("ОШИБКА: Файл shem.png не найден")
            self.schematic_label.setStyleSheet("background-color: lightgray; color: red;")
            self.schematic_label.setFixedSize(503, 397)

        left_layout.addWidget(self.schematic_label)

        # Абсолютное позиционирование поверх картинки (вычислено из Form1.designer.vb)
        # Координаты X = X_orig - 256, Y = Y_orig - 12

        # Давление (оригинал 268, 250 -> стало 12, 238)
        self.lbl_pump_press = QLabel("0.00", self.schematic_label)
        self.lbl_pump_press.setStyleSheet(
            "background-color: white; border: 2px solid black; font-weight: bold; font-size: 16px; padding: 2px;")
        self.lbl_pump_press.resize(80, 30)
        self.lbl_pump_press.move(12, 238)

        valve_style = "background-color: red; color: white; border: 1px solid black; padding: 2px; font-size: 10px;"

        # В клапан обратный (to_revers: 382, 108 -> стало 126, 96)
        self.ind_valve_rev_in = QLabel("В ОБРАТНЫЙ", self.schematic_label)
        self.ind_valve_rev_in.setStyleSheet(valve_style)
        self.ind_valve_rev_in.move(126, 96)

        # Из клапана прямой (from_forward: 555, 108 -> стало 299, 96)
        self.ind_valve_fwd_out = QLabel("ИЗ ПРЯМОГО", self.schematic_label)
        self.ind_valve_fwd_out.setStyleSheet(valve_style)
        self.ind_valve_fwd_out.move(299, 96)

        # В клапан прямой (to_forward: 382, 250 -> стало 126, 238)
        self.ind_valve_fwd_in = QLabel("В ПРЯМОЙ", self.schematic_label)
        self.ind_valve_fwd_in.setStyleSheet(valve_style)
        self.ind_valve_fwd_in.move(126, 238)

        # Из клапана обратный (from_revers: 555, 250 -> стало 299, 238)
        self.ind_valve_rev_out = QLabel("ИЗ ОБРАТНОГО", self.schematic_label)
        self.ind_valve_rev_out.setStyleSheet(valve_style)
        self.ind_valve_rev_out.move(299, 238)

        main_layout.addWidget(left_pane)

        # ================= ПРАВАЯ ПАНЕЛЬ (ГРАФИК И КНОПКИ) =================
        right_pane = QWidget()
        right_layout = QVBoxLayout(right_pane)

        # График
        self.plot_widget = pg.PlotWidget(title="Давление (атм)")
        self.plot_widget.setLabel('left', 'Давление', units='атм')
        self.plot_widget.setLabel('bottom', 'Время', units='с')
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_line = self.plot_widget.plot(pen=pg.mkPen('r', width=2))
        right_layout.addWidget(self.plot_widget, stretch=3)

        # Кнопки управления (выстроены горизонтально под графиком)
        control_layout = QHBoxLayout()

        self.btn_fwd = QPushButton("Прямой тест")
        self.btn_fwd.setStyleSheet("background-color: lightblue; height: 50px; font-weight: bold; font-size: 14px;")
        self.btn_fwd.clicked.connect(self.start_forward_test)

        self.btn_rev = QPushButton("Обратный тест")
        self.btn_rev.setStyleSheet("background-color: lightgreen; height: 50px; font-weight: bold; font-size: 14px;")
        self.btn_rev.clicked.connect(self.start_reverse_test)

        self.btn_stop = QPushButton("СТОП!!!")
        self.btn_stop.setStyleSheet(
            "background-color: red; color: white; height: 50px; font-weight: bold; font-size: 16px;")
        self.btn_stop.clicked.connect(self.stop_all)

        control_layout.addWidget(self.btn_fwd)
        control_layout.addWidget(self.btn_rev)
        control_layout.addWidget(self.btn_stop)

        right_layout.addLayout(control_layout)

        # Текстовое поле логов (textMemo)
        self.log_console = QTextEdit()
        self.log_console.setReadOnly(True)
        self.log_console.setFixedHeight(60)
        right_layout.addWidget(self.log_console)

        main_layout.addWidget(right_pane)
        self.log("Система готова. Ожидание команд.")

    def set_buttons_enabled(self, is_enabled):
        self.btn_fwd.setEnabled(is_enabled)
        self.btn_rev.setEnabled(is_enabled)
        alpha = "255" if is_enabled else "100"
        self.btn_fwd.setStyleSheet(
            f"background-color: rgba(173, 216, 230, {alpha}); height: 50px; font-weight: bold; font-size: 14px;")
        self.btn_rev.setStyleSheet(
            f"background-color: rgba(144, 238, 144, {alpha}); height: 50px; font-weight: bold; font-size: 14px;")

    def update_valve_ui(self, channel, state):
        style = "background-color: green; color: white; border: 1px solid black; padding: 2px; font-size: 10px;" if state else \
            "background-color: red; color: white; border: 1px solid black; padding: 2px; font-size: 10px;"

        if channel == self.DO_VALVE_FWD_IN:
            self.ind_valve_fwd_in.setStyleSheet(style)
        elif channel == self.DO_VALVE_REV_IN:
            self.ind_valve_rev_in.setStyleSheet(style)
        elif channel == self.DO_VALVE_FWD_OUT:
            self.ind_valve_fwd_out.setStyleSheet(style)
        elif channel == self.DO_VALVE_REV_OUT:
            self.ind_valve_rev_out.setStyleSheet(style)

    def init_modbus(self):
        self.modbus = ModbusSerialClient(
            port='COM3',
            baudrate=57600,
            timeout=0.1
        )
        if not DUMMY_MODE:
            if self.modbus.connect():
                self.log("Подключение к COM3 установлено.")
            else:
                self.log("ОШИБКА: Не удалось открыть COM3!")

    def log(self, text):
        t = time.strftime("%H:%M:%S")
        self.log_console.append(f"[{t}] {text}")

    # ================== АППАРАТНЫЙ УРОВЕНЬ ==================
    def write_do(self, channel, state):
        self.update_valve_ui(channel, state)
        if DUMMY_MODE: return
        try:
            self.modbus.write_coil(channel, state, slave=self.ADDR_DO)
        except Exception as e:
            self.log(f"Ошибка записи Modbus DO: {e}")

    def read_telemetry(self):
        if DUMMY_MODE:
            if self.state == "FWD_WAIT_OPEN":
                self.current_pressure += 0.1
                if self.current_pressure > 3.5: self.current_flow = 10
            elif self.state == "FWD_WAIT_CLOSE":
                self.current_pressure -= 0.1
                if self.current_pressure < 1.0: self.current_flow = 0
            elif self.state == "REV_TEST":
                self.current_pressure += 0.05
            else:
                self.current_pressure = max(0.0, self.current_pressure - 0.1)
            return

        try:
            res_ai = self.modbus.read_input_registers(address=5, count=1, slave=self.ADDR_AO)
            if not res_ai.isError():
                raw_val = res_ai.registers[0]
                self.current_pressure = (raw_val / 65535.0) * 6.0

            reg_flow = 10 if self.state in ["FWD_WAIT_OPEN", "FWD_WAIT_CLOSE"] else 11
            res_flow = self.modbus.read_holding_registers(address=reg_flow, count=1, slave=self.ADDR_FLOW)
            if not res_flow.isError():
                self.current_flow = res_flow.registers[0]
        except Exception:
            pass

    # ================== БИЗНЕС ЛОГИКА ==================
    def start_forward_test(self):
        self.set_buttons_enabled(False)
        self.log("--- СТАРТ: Прямой тест ---")
        self.time_data, self.pressure_data = [], []
        self.start_time = time.time()
        self.current_flow = 0

        self.write_do(self.DO_VALVE_FWD_IN, True)
        self.write_do(self.DO_VALVE_FWD_OUT, True)
        self.write_do(self.DO_VALVE_REV_IN, False)
        self.write_do(self.DO_VALVE_REV_OUT, False)

        QTimer.singleShot(500, lambda: self.write_do(self.DO_PUMP_START, True))
        self.state = "FWD_WAIT_OPEN"

    def start_reverse_test(self):
        self.set_buttons_enabled(False)
        self.log("--- СТАРТ: Обратный тест (30 сек) ---")
        self.time_data, self.pressure_data = [], []
        self.start_time = time.time()
        self.current_flow = 0

        self.write_do(self.DO_VALVE_FWD_IN, False)
        self.write_do(self.DO_VALVE_FWD_OUT, False)
        self.write_do(self.DO_VALVE_REV_IN, True)
        self.write_do(self.DO_VALVE_REV_OUT, True)

        QTimer.singleShot(500, lambda: self.write_do(self.DO_PUMP_START, True))
        self.state = "REV_TEST"

    def stop_all(self):
        if self.state != "IDLE":
            self.log("--- ОСТАНОВКА СИСТЕМЫ ---")

        self.write_do(self.DO_PUMP_START, False)
        self.write_do(self.DO_VALVE_FWD_IN, False)
        self.write_do(self.DO_VALVE_FWD_OUT, False)
        self.write_do(self.DO_VALVE_REV_IN, False)
        self.write_do(self.DO_VALVE_REV_OUT, False)

        self.state = "IDLE"
        self.set_buttons_enabled(True)

    # ================== ГЛАВНЫЙ ЦИКЛ ==================
    def main_loop(self):
        self.read_telemetry()
        self.lbl_pump_press.setText(f"{self.current_pressure:.2f}")

        if self.state != "IDLE" or self.current_pressure > 0.1:
            t_elapsed = time.time() - self.start_time if self.start_time > 0 else 0
            self.time_data.append(t_elapsed)
            self.pressure_data.append(self.current_pressure)
            self.time_data = self.time_data[-2000:]
            self.pressure_data = self.pressure_data[-2000:]
            self.plot_line.setData(self.time_data, self.pressure_data)

        if self.state == "FWD_WAIT_OPEN":
            if self.current_flow > 0:
                self.fixed_open = self.current_pressure
                self.log(f"ОТКРЫТИЕ зафиксировано: {self.fixed_open:.2f} атм")
                self.write_do(self.DO_PUMP_START, False)
                self.state = "FWD_WAIT_CLOSE"

        elif self.state == "FWD_WAIT_CLOSE":
            if self.current_flow == 0:
                self.log(f"ЗАКРЫТИЕ зафиксировано: {self.current_pressure:.2f} атм")
                self.log(f"Итог: Открытие {self.fixed_open:.2f} атм | Закрытие {self.current_pressure:.2f} атм")
                self.stop_all()

        elif self.state == "REV_TEST":
            t_elapsed = time.time() - self.start_time

            if self.current_flow > 0:
                self.log(f"БРАК! Протечка обнаружена на {t_elapsed:.1f} сек. Давление: {self.current_pressure:.2f} атм")
                self.stop_all()

            elif t_elapsed >= 30.0:
                if self.current_pressure > 2.0:
                    self.log(f"УСПЕХ! Клапан годен. Герметичность выдержана ({self.current_pressure:.2f} атм).")
                else:
                    self.log(f"БРАК / ОШИБКА! За 30 сек не набрано нужное давление ({self.current_pressure:.2f} атм).")
                self.stop_all()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ValveTestBench()
    window.show()
    sys.exit(app.exec())