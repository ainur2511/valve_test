import sys
import time
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QGridLayout, QPushButton, QTextEdit,
                             QLabel, QInputDialog, QLineEdit, QGroupBox)
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import QTimer, Qt
import pyqtgraph as pg
from pymodbus.client import ModbusSerialClient

DUMMY_MODE = True


class ValveTestBench(QMainWindow):
    def __init__(self):
        super().__init__()
        self.modbus = None
        self.port_ok = False
        self.service_mode = False
        self.is_auto_mode = False

        self.setWindowTitle("Стенд тестирования перепускных клапанов")
        self.resize(1300, 750)

        # --- Аппаратные настройки ---
        self.ADDR_DO = "01"   # I-7050D (DCON)
        self.ADDR_AO = "02"   # I-7017 (DCON)
        self.ADDR_FLOW = 86   # M-7041D (Modbus RTU)

        # Каналы DO (I-7050D)
        self.DO_VALVE_FWD_IN = 1
        self.DO_VALVE_REV_OUT = 2
        self.DO_VALVE_FWD_OUT = 3
        self.DO_VALVE_REV_IN = 4
        self.DO_PUMP_START = 5

        # Калибровка датчика давления (I-7017, канал 5) из оригинального VB.NET
        self.AI_PRESSURE_CH = 5
        self.MIN_MA = 3.86
        self.MAX_MA = 19.368
        self.MIN_P = 0.0
        self.MAX_P = 6.0

        # Каналы расходомера (M-7041D, вход 10)
        self.FLOW_INPUT_REG = 10
        self.FLOW_CLEAR_COIL = 522

        # Текущие состояния регистров для диагностики
        self.do_states = {
            self.DO_VALVE_FWD_IN: False,
            self.DO_VALVE_REV_OUT: False,
            self.DO_VALVE_FWD_OUT: False,
            self.DO_VALVE_REV_IN: False,
            self.DO_PUMP_START: False,
        }
        self.raw_pressure_ma = 0.0
        self.current_pressure = 0.0
        self.current_flow = 0
        self.fixed_open = 0.0

        # --- Переменные состояния ---
        # Возможные: IDLE, FWD_WAIT_OPEN, FWD_WAIT_CLOSE, AUTO_PAUSE, REV_TEST
        self.state = "IDLE"
        self.start_time = 0
        self.time_data = []
        self.pressure_data = []

        self.init_ui()
        self.init_modbus()

        self.timer = QTimer()
        self.timer.timeout.connect(self.main_loop)
        self.timer.start(200)

    def _create_led(self):
        led = QLabel()
        led.setFixedSize(16, 16)
        led.setStyleSheet(
            "background-color: #dc3545; border-radius: 8px; border: 1px solid #333;"
        )
        return led

    def _set_led_state(self, led, state):
        color = "#28a745" if state else "#dc3545"
        led.setStyleSheet(
            f"background-color: {color}; border-radius: 8px; border: 1px solid #333;"
        )

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # ================= ЛЕВАЯ ПАНЕЛЬ (МНЕМОСХЕМА + СЕРВИС) =================
        left_pane = QWidget()
        scale = 1.25
        schem_w = int(503 * scale)
        schem_h = int(397 * scale)

        left_pane.setFixedWidth(schem_w + 20)
        left_layout = QVBoxLayout(left_pane)
        left_layout.setContentsMargins(5, 5, 5, 5)
        left_layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        self.schematic_label = QLabel()
        try:
            pixmap = QPixmap("shem.png").scaled(schem_w, schem_h, Qt.AspectRatioMode.IgnoreAspectRatio)
            self.schematic_label.setPixmap(pixmap)
            self.schematic_label.setFixedSize(schem_w, schem_h)
        except Exception:
            self.schematic_label.setText("ОШИБКА: Файл shem.png не найден")
            self.schematic_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.schematic_label.setStyleSheet("background-color: lightgray; color: red; font-size: 16px;")
            self.schematic_label.setFixedSize(schem_w, schem_h)

        left_layout.addWidget(self.schematic_label)

        # Индикатор давления (отцентрованный текст)
        self.lbl_pump_press = QLabel("0.00", self.schematic_label)
        self.lbl_pump_press.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_pump_press.setStyleSheet(
            "background-color: white; border: 2px solid black; font-weight: bold; font-size: 18px;"
        )
        self.lbl_pump_press.resize(int(85 * scale), int(32 * scale))
        self.lbl_pump_press.move(int(12 * scale), int(238 * scale))

        valve_style = "background-color: red; color: white; border: 1px solid black; padding: 3px; font-size: 11px; font-weight: bold;"

        self.ind_valve_rev_in = QLabel("В ОБРАТНЫЙ", self.schematic_label)
        self.ind_valve_rev_in.setStyleSheet(valve_style)
        self.ind_valve_rev_in.move(int(126 * scale), int(96 * scale))

        self.ind_valve_fwd_out = QLabel("ИЗ ПРЯМОГО", self.schematic_label)
        self.ind_valve_fwd_out.setStyleSheet(valve_style)
        self.ind_valve_fwd_out.move(int(299 * scale), int(96 * scale))

        self.ind_valve_fwd_in = QLabel("В ПРЯМОЙ", self.schematic_label)
        self.ind_valve_fwd_in.setStyleSheet(valve_style)
        self.ind_valve_fwd_in.move(int(126 * scale), int(238 * scale))

        self.ind_valve_rev_out = QLabel("ИЗ ОБРАТНОГО", self.schematic_label)
        self.ind_valve_rev_out.setStyleSheet(valve_style)
        self.ind_valve_rev_out.move(int(299 * scale), int(238 * scale))

        # Кнопка сервисного режима
        self.btn_service = QPushButton("Сервисный режим")
        self.btn_service.setStyleSheet("height: 36px; font-size: 13px; font-weight: bold;")
        self.btn_service.clicked.connect(self.toggle_service_mode)
        left_layout.addWidget(self.btn_service)

        # --- Панель диагностики регистров ---
        self.service_panel = QGroupBox("Диагностика регистров (Сервисный режим)")
        self.service_panel.setStyleSheet(
            "QGroupBox { font-weight: bold; font-size: 13px; background-color: #f8f9fa; border: 1px solid #ccc; margin-top: 8px; padding-top: 10px; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }"
        )
        srv_layout = QVBoxLayout(self.service_panel)
        srv_layout.setSpacing(6)

        # Дискретные выходы (I-7050D, DCON #01)
        do_grid = QGridLayout()
        do_items = [
            (self.DO_VALVE_FWD_IN, "DO1 (В прям.)"),
            (self.DO_VALVE_REV_OUT, "DO2 (Из обр.)"),
            (self.DO_VALVE_FWD_OUT, "DO3 (Из прям.)"),
            (self.DO_VALVE_REV_IN, "DO4 (В обр.)"),
            (self.DO_PUMP_START, "DO5 (Насос)"),
        ]
        self.do_leds = {}
        for idx, (ch, name) in enumerate(do_items):
            cell = QHBoxLayout()
            cell.setContentsMargins(0, 0, 8, 0)
            led = self._create_led()
            lbl = QLabel(name)
            lbl.setStyleSheet("font-size: 12px; font-weight: normal;")
            cell.addWidget(led)
            cell.addWidget(lbl)
            cell.addStretch()
            self.do_leds[ch] = led
            do_grid.addLayout(cell, idx // 3, idx % 3)

        srv_layout.addLayout(do_grid)

        # Аналоговый вход (I-7017, DCON #025)
        self.lbl_srv_ai = QLabel("AI5 (I-7017, #025): Ток = 0.000 мА | Давление = 0.00 атм")
        self.lbl_srv_ai.setStyleSheet(
            "background-color: #e9ecef; padding: 4px 6px; font-size: 13px; font-weight: bold; border-radius: 3px;"
        )
        srv_layout.addWidget(self.lbl_srv_ai)

        # Расходомер (M-7041D, Modbus ID 86, Input Reg 10)
        self.lbl_srv_flow = QLabel("Расходомер (M-7041D, ID 86, Reg 10): 0 ед.")
        self.lbl_srv_flow.setStyleSheet(
            "background-color: #fff3cd; padding: 4px 6px; font-size: 13px; font-weight: bold; border-radius: 3px;"
        )
        srv_layout.addWidget(self.lbl_srv_flow)

        self.service_panel.setVisible(False)
        left_layout.addWidget(self.service_panel)

        main_layout.addWidget(left_pane)

        # ================= ПРАВАЯ ПАНЕЛЬ (ГРАФИК, КНОПКИ, ЛОГИ) =================
        right_pane = QWidget()
        right_layout = QVBoxLayout(right_pane)

        self.plot_widget = pg.PlotWidget(title="Давление (атм)")
        self.plot_widget.setLabel('left', 'Давление', units='атм')
        self.plot_widget.setLabel('bottom', 'Время', units='с')
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_line = self.plot_widget.plot(pen=pg.mkPen('r', width=2))
        right_layout.addWidget(self.plot_widget, stretch=2)

        control_layout = QHBoxLayout()

        self.btn_auto = QPushButton("Авто-тест")
        self.btn_auto.clicked.connect(self.start_auto_test)

        self.btn_fwd = QPushButton("Прямой тест")
        self.btn_fwd.clicked.connect(self.start_forward_test)

        self.btn_rev = QPushButton("Обратный тест")
        self.btn_rev.clicked.connect(self.start_reverse_test)

        self.btn_stop = QPushButton("СТОП!!!")
        self.btn_stop.setStyleSheet("background-color: red; color: white; height: 50px; font-weight: bold; font-size: 16px;")
        self.btn_stop.clicked.connect(self.stop_all)

        control_layout.addWidget(self.btn_auto)
        control_layout.addWidget(self.btn_fwd)
        control_layout.addWidget(self.btn_rev)
        control_layout.addWidget(self.btn_stop)
        right_layout.addLayout(control_layout)

        log_header_layout = QHBoxLayout()
        lbl_log_title = QLabel("Журнал событий:")
        lbl_log_title.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.btn_clear_log = QPushButton("Очистить сообщения")
        self.btn_clear_log.setStyleSheet("padding: 4px 10px; font-size: 13px;")
        self.btn_clear_log.clicked.connect(self.clear_log)

        log_header_layout.addWidget(lbl_log_title)
        log_header_layout.addStretch()
        log_header_layout.addWidget(self.btn_clear_log)
        right_layout.addLayout(log_header_layout)

        self.log_console = QTextEdit()
        self.log_console.setReadOnly(True)
        self.log_console.setStyleSheet("font-size: 15px;")
        right_layout.addWidget(self.log_console, stretch=2)

        main_layout.addWidget(right_pane, stretch=1)
        self.set_buttons_enabled(True)
        self.log("Система готова. Ожидание команд.")

    def toggle_service_mode(self):
        if self.service_mode:
            self.service_mode = False
            self.service_panel.setVisible(False)
            self.btn_service.setText("Сервисный режим")
            self.log("Сервисный режим отключен.")
        else:
            pwd, ok = QInputDialog.getText(
                self, "Сервисный режим", "Введите пароль:", QLineEdit.EchoMode.Password
            )
            if ok and pwd == "0000":
                self.service_mode = True
                self.service_panel.setVisible(True)
                self.btn_service.setText("Выйти из сервиса")
                self.log("Сервисный режим активирован.")
            elif ok:
                self.log("Неверный пароль сервисного режима!", is_error=True)

    def clear_log(self):
        self.log_console.clear()

    def set_buttons_enabled(self, is_enabled):
        self.btn_auto.setEnabled(is_enabled)
        self.btn_fwd.setEnabled(is_enabled)
        self.btn_rev.setEnabled(is_enabled)
        alpha = "255" if is_enabled else "100"
        self.btn_auto.setStyleSheet(f"background-color: rgba(255, 215, 0, {alpha}); height: 50px; font-weight: bold; font-size: 14px;")
        self.btn_fwd.setStyleSheet(f"background-color: rgba(173, 216, 230, {alpha}); height: 50px; font-weight: bold; font-size: 14px;")
        self.btn_rev.setStyleSheet(f"background-color: rgba(144, 238, 144, {alpha}); height: 50px; font-weight: bold; font-size: 14px;")

    def update_valve_ui(self, channel, state):
        if channel in self.do_leds:
            self._set_led_state(self.do_leds[channel], state)

        color = "green" if state else "red"
        style = f"background-color: {color}; color: white; border: 1px solid black; padding: 3px; font-size: 11px; font-weight: bold;"

        if channel == self.DO_VALVE_FWD_IN:
            self.ind_valve_fwd_in.setStyleSheet(style)
        elif channel == self.DO_VALVE_REV_IN:
            self.ind_valve_rev_in.setStyleSheet(style)
        elif channel == self.DO_VALVE_FWD_OUT:
            self.ind_valve_fwd_out.setStyleSheet(style)
        elif channel == self.DO_VALVE_REV_OUT:
            self.ind_valve_rev_out.setStyleSheet(style)

    def init_modbus(self):
        if DUMMY_MODE:
            self.port_ok = True
            self.log("Запущено в тестовом режиме (DUMMY_MODE = True).")
            return

        self.modbus = ModbusSerialClient(
            port='COM11',
            baudrate=57600,
            timeout=1
        )
        if self.modbus.connect():
            self.port_ok = True
            self.log("Подключение к COM11 (DCON + Modbus RTU) установлено.")
        else:
            self.port_ok = False
            self.set_buttons_enabled(False)
            self.log("ОШИБКА: Не удалось открыть COM3! Запуск тестов заблокирован.", is_error=True)

    def log(self, text, is_error=False, is_success=False):
        t = time.strftime("%H:%M:%S")
        if is_error:
            html = f'<span style="color: #d90000; font-weight: bold;">[{t}] {text}</span>'
        elif is_success:
            html = f'<span style="color: #008000; font-weight: bold;">[{t}] {text}</span>'
        else:
            html = f'<span>[{t}] {text}</span>'
        self.log_console.append(html)

    # ================== АППАРАТНЫЙ УРОВЕНЬ (DCON + MODBUS) ==================
    def send_dcon(self, cmd: str) -> str:
        """Отправка DCON-команды через уже открытый COM-порт pymodbus"""
        if DUMMY_MODE or not self.port_ok:
            return ""
        try:
            ser = self.modbus.socket
            ser.reset_input_buffer()
            ser.write(f"{cmd}\r".encode("ascii"))
            return ser.read_until(b"\r").decode("ascii", errors="ignore").strip()
        except Exception as e:
            self.log(f"Ошибка DCON ({cmd}): {e}", is_error=True)
            return ""

    def write_do(self, channel, state):
        """Управление реле I-7050D по DCON (#011C0V)"""
        self.do_states[channel] = state
        self.update_valve_ui(channel, state)
        if DUMMY_MODE:
            return
        val = 1 if state else 0
        self.send_dcon(f"#{self.ADDR_DO}1{channel}0{val}")

    def clear_flow_counter(self):
        """Сброс счетчика 10-го входа на M-7041D (Modbus Coil 522)"""
        self.current_flow = 0
        if DUMMY_MODE:
            return
        try:
            self.modbus.write_coil(self.FLOW_CLEAR_COIL, True, device_id=self.ADDR_FLOW)
        except Exception as e:
            self.log(f"Ошибка сброса счетчика (Coil {self.FLOW_CLEAR_COIL}): {e}", is_error=True)

    def read_telemetry(self):
        if DUMMY_MODE:
            if self.state == "FWD_WAIT_OPEN":
                self.current_pressure += 0.08
                if self.current_pressure > 1.5:
                    self.current_flow += 5
            elif self.state == "FWD_WAIT_CLOSE":
                self.current_pressure -= 0.08
                if self.current_pressure < 0.6:
                    self.current_flow = 0
            elif self.state == "REV_TEST":
                self.current_pressure = min(2.5, self.current_pressure + 0.05)
            else:
                self.current_pressure = max(0.0, self.current_pressure - 0.1)

            self.raw_pressure_ma = self.MIN_MA + (self.current_pressure / self.MAX_P) * (self.MAX_MA - self.MIN_MA)
            return

        if not self.port_ok:
            return

        # 1. Чтение давления с I-7017 (DCON #025 -> возвращает мА)
        resp = self.send_dcon(f"#{self.ADDR_AO}{self.AI_PRESSURE_CH}")
        if resp.startswith(">"):
            try:
                ma = float(resp[1:])
                self.raw_pressure_ma = ma
                # Калибровочная формула из оригинального VB.NET
                self.current_pressure = round(
                    abs(ma - self.MIN_MA) * (self.MAX_P - self.MIN_P) / (self.MAX_MA - self.MIN_MA), 2
                )
            except ValueError:
                pass

        # 2. Чтение счетчика расходомера с M-7041D (Modbus RTU, Input Register 10)
        try:
            res_flow = self.modbus.read_input_registers(
                address=self.FLOW_INPUT_REG, count=1, device_id=self.ADDR_FLOW
            )
            if not res_flow.isError():
                self.current_flow = res_flow.registers[0]
        except Exception:
            pass

    # ================== БИЗНЕС ЛОГИКА ==================
    def start_auto_test(self):
        if not self.port_ok:
            return
        self.is_auto_mode = True
        self.log("=== СТАРТ: АВТОМАТИЧЕСКИЙ ТЕСТ (Прямой -> Обратный) ===")
        self.start_forward_test(keep_auto=True)

    def start_forward_test(self, keep_auto=False):
        if not self.port_ok:
            return
        if not keep_auto:
            self.is_auto_mode = False
        self.set_buttons_enabled(False)
        self.log("--- СТАРТ: Прямой тест ---")
        self.time_data, self.pressure_data = [], []
        self.start_time = time.time()

        self.clear_flow_counter()

        self.write_do(self.DO_VALVE_FWD_IN, True)
        self.write_do(self.DO_VALVE_FWD_OUT, True)
        self.write_do(self.DO_VALVE_REV_IN, False)
        self.write_do(self.DO_VALVE_REV_OUT, False)

        QTimer.singleShot(500, lambda: self.write_do(self.DO_PUMP_START, True))
        self.state = "FWD_WAIT_OPEN"

    def start_reverse_test(self, keep_auto=False):
        if not self.port_ok:
            return
        if not keep_auto:
            self.is_auto_mode = False
        self.set_buttons_enabled(False)
        self.log("--- СТАРТ: Обратный тест (30 сек) ---")
        self.time_data, self.pressure_data = [], []
        self.start_time = time.time()

        self.clear_flow_counter()

        self.write_do(self.DO_VALVE_FWD_IN, False)
        self.write_do(self.DO_VALVE_FWD_OUT, False)
        self.write_do(self.DO_VALVE_REV_IN, True)
        self.write_do(self.DO_VALVE_REV_OUT, True)

        QTimer.singleShot(500, lambda: self.write_do(self.DO_PUMP_START, True))
        self.state = "REV_TEST"

    def _continue_auto_reverse(self):
        if self.state == "AUTO_PAUSE" and self.is_auto_mode:
            self.start_reverse_test(keep_auto=True)

    def stop_all(self):
        if self.state != "IDLE":
            self.log("--- ОСТАНОВКА СИСТЕМЫ ---")

        self.write_do(self.DO_PUMP_START, False)
        self.write_do(self.DO_VALVE_FWD_IN, False)
        self.write_do(self.DO_VALVE_FWD_OUT, False)
        self.write_do(self.DO_VALVE_REV_IN, False)
        self.write_do(self.DO_VALVE_REV_OUT, False)

        self.state = "IDLE"
        self.is_auto_mode = False
        self.set_buttons_enabled(self.port_ok)

    # ================== ГЛАВНЫЙ ЦИКЛ ==================
    def main_loop(self):
        self.read_telemetry()
        self.lbl_pump_press.setText(f"{self.current_pressure:.2f}")

        if self.service_mode:
            self.lbl_srv_ai.setText(
                f"AI5 (I-7017, #025): Ток = {self.raw_pressure_ma:.3f} мА | Давление = {self.current_pressure:.2f} атм"
            )
            self.lbl_srv_flow.setText(
                f"Расходомер (M-7041D, ID 86, Reg 10): {self.current_flow} ед."
            )

        if self.state not in ["IDLE", "AUTO_PAUSE"] or self.current_pressure > 0.1:
            t_elapsed = time.time() - self.start_time if self.start_time > 0 else 0
            self.time_data.append(t_elapsed)
            self.pressure_data.append(self.current_pressure)
            self.time_data = self.time_data[-2000:]
            self.pressure_data = self.pressure_data[-2000:]
            self.plot_line.setData(self.time_data, self.pressure_data)

        if self.state == "FWD_WAIT_OPEN":
            t_elapsed = time.time() - self.start_time

            if self.current_flow > 0:
                self.fixed_open = self.current_pressure
                self.log(f"ОТКРЫТИЕ зафиксировано: {self.fixed_open:.2f} атм")
                self.write_do(self.DO_PUMP_START, False)
                self.state = "FWD_WAIT_CLOSE"
            elif t_elapsed >= 30.0 and self.current_pressure > 0.1:
                self.log("ОШИБКА: Нет расхода в течение 30 сек при наличии давления! Проверьте исправность расходомера.", is_error=True)
                self.stop_all()

        elif self.state == "FWD_WAIT_CLOSE":
            if self.current_flow == 0:
                closing_pressure = self.current_pressure
                self.log(f"ЗАКРЫТИЕ зафиксировано: {closing_pressure:.2f} атм")
                self.log(f"Итог прямого теста: Открытие {self.fixed_open:.2f} атм | Закрытие {closing_pressure:.2f} атм")

                open_ok = 0.2 <= self.fixed_open <= 2.0
                close_ok = closing_pressure >= 0.2

                if open_ok and close_ok:
                    self.log("РЕЗУЛЬТАТ ПРЯМОГО ТЕСТА: УСПЕХ! Клапан в допуске.", is_success=True)
                    if self.is_auto_mode:
                        self.write_do(self.DO_PUMP_START, False)
                        self.write_do(self.DO_VALVE_FWD_IN, False)
                        self.write_do(self.DO_VALVE_FWD_OUT, False)
                        self.state = "AUTO_PAUSE"
                        self.log("Пауза 3 сек перед запуском обратного теста...")
                        QTimer.singleShot(3000, self._continue_auto_reverse)
                    else:
                        self.stop_all()
                else:
                    self.log("РЕЗУЛЬТАТ ПРЯМОГО ТЕСТА: БРАК! Клапан не прошел критерии.", is_error=True)
                    if self.is_auto_mode:
                        self.log("Автоматический тест прерван из-за провала прямого теста.", is_error=True)
                    self.stop_all()

        elif self.state == "REV_TEST":
            t_elapsed = time.time() - self.start_time

            if self.current_flow > 0:
                self.log(f"РЕЗУЛЬТАТ ОБРАТНОГО ТЕСТА: БРАК! Протечка клапана на {t_elapsed:.1f} сек ({self.current_pressure:.2f} атм).", is_error=True)
                self.stop_all()

            elif t_elapsed >= 30.0:
                if self.current_pressure >= 2.0:
                    self.log(f"РЕЗУЛЬТАТ ОБРАТНОГО ТЕСТА: УСПЕХ! Клапан герметичен ({self.current_pressure:.2f} атм).", is_success=True)
                else:
                    self.log(f"ТЕСТ ПРОВАЛЕН: Недостаточное давление ({self.current_pressure:.2f} атм). Проведите ревизию насоса и системы!", is_error=True)
                self.stop_all()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ValveTestBench()
    window.show()
    sys.exit(app.exec())