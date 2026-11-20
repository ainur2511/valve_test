import sys
import os

def resource_path(filename: str) -> str:
    """Путь к ресурсам (shem.png), вшитым в exe"""
    base = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base, filename)

def app_dir() -> str:
    """Папка, в которой лежит exe (или main.py) — для config.ini"""
    if getattr(sys, "frozen", False):
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.abspath(__file__))
import time
import queue
import logging
from serial import SerialException
from serial.tools import list_ports
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QGridLayout, QPushButton, QTextEdit,
                             QLabel, QInputDialog, QLineEdit, QGroupBox,
                             QComboBox)
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import QTimer, Qt, QSettings, QThread, pyqtSignal
import pyqtgraph as pg
from pymodbus.client import ModbusSerialClient

# Убираем спам pymodbus из консоли Python
logging.getLogger("pymodbus").setLevel(logging.CRITICAL)

# Поставьте True для проверки без подключенного стенда
DUMMY_MODE = True


class CommWorker(QThread):
    """Фоновый поток для всей работы с COM-портом (DCON + Modbus RTU + Watchdog слейвов)"""
    telemetry_updated = pyqtSignal(float, float, int, int, int)  # (raw_ma, pressure_atm, flow_fwd, flow_rev, pedal)
    connection_changed = pyqtSignal(bool, str, bool)             # (is_connected, message, is_initial)
    slaves_status_changed = pyqtSignal(bool, bool, bool)         # (do_ok, ai_ok, flow_ok)

    def __init__(self, port, baudrate, addr_do, addr_ao, ai_ch,
                 addr_flow, flow_reg_start, flow_fwd_clear_coil, flow_rev_clear_coil,
                 min_ma, max_ma, min_p, max_p, max_missed=5):
        super().__init__()
        self.com_port = port
        self.baudrate = baudrate
        self.addr_do = addr_do
        self.addr_ao = addr_ao
        self.ai_ch = ai_ch
        self.addr_flow = addr_flow
        self.flow_reg_start = flow_reg_start
        self.flow_fwd_clear_coil = flow_fwd_clear_coil
        self.flow_rev_clear_coil = flow_rev_clear_coil
        self.min_ma = min_ma
        self.max_ma = max_ma
        self.min_p = min_p
        self.max_p = max_p
        self.max_missed = max_missed

        self.cmd_queue = queue.Queue()
        self.modbus = None
        self.connected = False
        self._running = True
        self._initial_attempt = True
        self.reconnect_interval = 2.0
        self.last_reconnect_time = 0.0

        # Счетчики пропусков и статусы слейвов
        self.err_do = 0
        self.err_ai = 0
        self.err_flow = 0
        self.do_ok = False
        self.ai_ok = False
        self.flow_ok = False

        self.raw_ma = 0.0
        self.pressure = 0.0
        self.flow_fwd = 0
        self.flow_rev = 0
        self.pedal_count = 0

    def enqueue_do(self, channel: int, state: bool):
        self.cmd_queue.put(("write_do", channel, state))

    def enqueue_clear_flow(self, coil: int):
        self.cmd_queue.put(("clear_flow", coil))

    def enqueue_change_port(self, new_port: str):
        self.cmd_queue.put(("change_port", new_port))

    def stop(self):
        self._running = False

    def _reset_slaves_status(self):
        self.err_do = 0
        self.err_ai = 0
        self.err_flow = 0
        if self.do_ok or self.ai_ok or self.flow_ok:
            self.do_ok = False
            self.ai_ok = False
            self.flow_ok = False
            self.slaves_status_changed.emit(False, False, False)

    def _close_port(self):
        if self.modbus is not None:
            try:
                self.modbus.close()
            except Exception:
                pass
            self.modbus = None

    def _try_connect(self):
        self._close_port()
        try:
            self.modbus = ModbusSerialClient(
                port=self.com_port,
                baudrate=self.baudrate,
                timeout=0.1,
                retries=0
            )
            ok = self.modbus.connect()
        except Exception:
            ok = False

        if ok:
            self.connected = True
            self.err_do = self.err_ai = self.err_flow = 0
            self.do_ok = self.ai_ok = self.flow_ok = False
            msg = (f"Порт {self.com_port} открыт. Проверка связи с модулями..."
                   if self._initial_attempt
                   else f"Порт {self.com_port} восстановлен. Проверка связи с модулями...")
            self.connection_changed.emit(True, msg, self._initial_attempt)
            self.slaves_status_changed.emit(False, False, False)
            self._initial_attempt = False
        else:
            self.connected = False
            if self._initial_attempt:
                msg = (f"ОШИБКА: Не удалось открыть {self.com_port}! "
                       f"Подключите кабель или выберите другой порт в сервисном режиме.")
                self.connection_changed.emit(False, msg, True)
                self._initial_attempt = False

    def _handle_disconnect(self):
        if not self.connected:
            return
        self.connected = False
        self.last_reconnect_time = time.time()
        self._close_port()
        self._reset_slaves_status()
        while not self.cmd_queue.empty():
            try:
                item = self.cmd_queue.get_nowait()
                if item[0] == "change_port":
                    self.com_port = item[1]
            except queue.Empty:
                break
        self.connection_changed.emit(
            False,
            f"ПОТЕРЯНА СВЯЗЬ С {self.com_port}! Ожидание переподключения...",
            False
        )

    def _send_dcon(self, cmd: str) -> str:
        ser = self.modbus.socket
        if ser is None or not ser.is_open:
            raise SerialException("Port closed")
        ser.reset_input_buffer()
        ser.write(f"{cmd}\r".encode("ascii"))
        return ser.read_until(b"\r").decode("ascii", errors="ignore").strip()

    def _process_commands(self):
        while not self.cmd_queue.empty():
            try:
                cmd = self.cmd_queue.get_nowait()
            except queue.Empty:
                break

            action = cmd[0]
            if action == "change_port":
                self.com_port = cmd[1]
                self.connected = False
                self._initial_attempt = True
                self._close_port()
                self._reset_slaves_status()
                self.last_reconnect_time = 0.0
                return

            if not self.connected:
                continue

            try:
                if action == "write_do":
                    _, channel, state = cmd
                    val = 1 if state else 0
                    self._send_dcon(f"#{self.addr_do}1{channel}0{val}")
                elif action == "clear_flow":
                    _, coil = cmd
                    self.modbus.write_coil(coil, True, device_id=self.addr_flow)
                    if coil == self.flow_fwd_clear_coil:
                        self.flow_fwd = 0
                    elif coil == self.flow_rev_clear_coil:
                        self.flow_rev = 0
            except (SerialException, OSError):
                self._handle_disconnect()
                return
            except Exception:
                pass

    def run(self):
        while self._running:
            self._process_commands()

            if not self.connected:
                now = time.time()
                if now - self.last_reconnect_time >= self.reconnect_interval:
                    self.last_reconnect_time = now
                    self._try_connect()
                self.msleep(100)
                continue

            prev_status = (self.do_ok, self.ai_ok, self.flow_ok)

            try:
                # 1. Проверка живости I-7050D (DCON $016 -> ответ вида !XXXX)
                resp_do = self._send_dcon(f"${self.addr_do}6")
                if resp_do.startswith("!"):
                    self.err_do = 0
                    self.do_ok = True
                else:
                    self.err_do += 1
                    if self.err_do >= self.max_missed:
                        self.do_ok = False

                # 2. Опрос давления I-7017 (DCON #025 -> ответ вида >+04.123)
                resp_ai = self._send_dcon(f"#{self.addr_ao}{self.ai_ch}")
                ai_parsed = False
                if resp_ai.startswith(">"):
                    try:
                        ma = float(resp_ai[1:])
                        self.raw_ma = ma
                        self.pressure = round(
                            abs(ma - self.min_ma) * (self.max_p - self.min_p) / (self.max_ma - self.min_ma), 2
                        )
                        ai_parsed = True
                    except ValueError:
                        pass

                if ai_parsed:
                    self.err_ai = 0
                    self.ai_ok = True
                else:
                    self.err_ai += 1
                    if self.err_ai >= self.max_missed:
                        self.ai_ok = False

                # 3. Опрос счетчиков M-7041D: Reg 10 (прямой), Reg 11 (обратный), Reg 12 (педаль)
                res_flow = self.modbus.read_input_registers(
                    address=self.flow_reg_start, count=3, device_id=self.addr_flow
                )
                if not res_flow.isError():
                    self.flow_fwd = res_flow.registers[0]
                    self.flow_rev = res_flow.registers[1]
                    self.pedal_count = res_flow.registers[2]
                    self.err_flow = 0
                    self.flow_ok = True
                else:
                    self.err_flow += 1
                    if self.err_flow >= self.max_missed:
                        self.flow_ok = False

                # Уведомляем UI, если статус любого из модулей изменился
                curr_status = (self.do_ok, self.ai_ok, self.flow_ok)
                if curr_status != prev_status:
                    self.slaves_status_changed.emit(*curr_status)

                self.telemetry_updated.emit(
                    self.raw_ma, self.pressure, self.flow_fwd, self.flow_rev, self.pedal_count
                )

            except (SerialException, OSError):
                self._handle_disconnect()
            except Exception:
                pass

            self.msleep(150)

        self._close_port()


class ValveTestBench(QMainWindow):
    def __init__(self):
        super().__init__()
        self.worker = None
        self.port_ok = False
        self.do_ok = False
        self.ai_ok = False
        self.flow_ok = False
        self.service_mode = False
        self.is_auto_mode = False
        self.ignore_flow_until = 0.0

        # Загрузка сохраненного COM-порта из config.ini
        self.settings = QSettings(os.path.join(app_dir(), "config.ini"), QSettings.Format.IniFormat)
        self.COM_PORT = str(self.settings.value("com_port", "COM3"))
        self.BAUDRATE = 57600
        self.MAX_MISSED_POLLS = 5

        self.setWindowTitle("Стенд тестирования перепускных клапанов")
        self.resize(1300, 800)

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

        self.DO_NAMES = {
            self.DO_VALVE_FWD_IN: "DO1 (В прям.)",
            self.DO_VALVE_REV_OUT: "DO2 (Из обр.)",
            self.DO_VALVE_FWD_OUT: "DO3 (Из прям.)",
            self.DO_VALVE_REV_IN: "DO4 (В обр.)",
            self.DO_PUMP_START: "DO5 (Насос)",
        }

        # Калибровка датчика давления (I-7017, канал 5)
        self.AI_PRESSURE_CH = 5
        self.MIN_MA = 3.86
        self.MAX_MA = 19.368
        self.MIN_P = 0.0
        self.MAX_P = 6.0

        # Каналы M-7041D: вход 10 — прямой, вход 11 — обратный, вход 12 — педаль
        self.FLOW_FWD_REG = 10
        self.FLOW_REV_REG = 11
        self.PEDAL_REG = 12
        self.FLOW_FWD_CLEAR_COIL = 522
        self.FLOW_REV_CLEAR_COIL = 523

        # Настройки педали (антидребезг 1.0 сек)
        self.PEDAL_DEBOUNCE_TIME = 1.0
        self.pedal_count = 0
        self.last_pedal_count = None
        self.pedal_lock_until = 0.0

        # Настройки прямого теста (фиксация остановки потока)
        self.FLOW_STOP_TIMEOUT = 1.5   # Сек без новых импульсов = клапан закрылся
        self.last_flow_val = 0
        self.last_flow_time = 0.0
        self.last_flow_pressure = 0.0

        # Настройки переходов и обратного теста
        self.AUTO_PAUSE_TIME = 5.0     # Пауза после падения давления ниже 0.2 атм (сек)
        self.auto_pause_start_time = None
        self.REV_STABILIZE_TIME = 5.0  # Стабилизация (сек)
        self.REV_TEST_TIME = 30.0      # Контроль герметичности (сек)
        self.rev_flow_baseline = 0     # Запомненное значение счетчика (вход 11) после стабилизации
        self.rev_test_start_time = 0.0

        # Текущие состояния регистров для диагностики
        self.do_states = {ch: False for ch in self.DO_NAMES}
        self.raw_pressure_ma = 0.0
        self.current_pressure = 0.0
        self.flow_fwd = 0
        self.flow_rev = 0
        self.fixed_open = 0.0

        # --- Переменные состояния ---
        # Возможные: IDLE, FWD_WAIT_OPEN, FWD_WAIT_CLOSE, AUTO_PAUSE, REV_STABILIZE, REV_TEST
        self.state = "IDLE"
        self.start_time = 0
        self.time_data = []
        self.pressure_data = []

        self.init_ui()
        self.init_comm()

        # Таймер UI и бизнес-логики
        self.timer = QTimer()
        self.timer.timeout.connect(self.main_loop)
        self.timer.start(200)

    def _is_system_ready(self) -> bool:
        return self.port_ok and self.do_ok and self.ai_ok and self.flow_ok

    def _update_status_bar(self):
        if DUMMY_MODE:
            self.statusBar().showMessage("Режим симуляции (DUMMY_MODE) | Все модули: OK")
            return
        if not self.port_ok:
            self.statusBar().showMessage(f"{self.COM_PORT}: НЕТ ПОДКЛЮЧЕНИЯ (ожидание кабеля...)")
            return

        s_do = "OK" if self.do_ok else "НЕТ СВЯЗИ"
        s_ai = "OK" if self.ai_ok else "НЕТ СВЯЗИ"
        s_fl = "OK" if self.flow_ok else "НЕТ СВЯЗИ"
        self.statusBar().showMessage(
            f"{self.COM_PORT}: OK  |  I-7050D (Реле): {s_do}  |  I-7017 (Давление): {s_ai}  |  M-7041D (Расход): {s_fl}"
        )

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

    def _update_manual_buttons(self):
        """Ручное управление доступно только в режиме простоя при открытом COM-порте"""
        can_manual = self.port_ok and (self.state == "IDLE")
        for btn in self.manual_do_btns.values():
            btn.setEnabled(can_manual)

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
            pixmap = QPixmap(resource_path("shem.png")).scaled(schem_w, schem_h, Qt.AspectRatioMode.IgnoreAspectRatio)
            self.schematic_label.setPixmap(pixmap)
            self.schematic_label.setFixedSize(schem_w, schem_h)
        except Exception:
            self.schematic_label.setText("ОШИБКА: Файл shem.png не найден")
            self.schematic_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.schematic_label.setStyleSheet("background-color: lightgray; color: red; font-size: 16px;")
            self.schematic_label.setFixedSize(schem_w, schem_h)

        left_layout.addWidget(self.schematic_label)

        # Индикатор давления
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

        # --- Панель диагностики и ручного управления ---
        self.service_panel = QGroupBox("Диагностика и ручное управление (Сервисный режим)")
        self.service_panel.setStyleSheet(
            "QGroupBox { font-weight: bold; font-size: 13px; background-color: #f8f9fa; border: 1px solid #ccc; margin-top: 8px; padding-top: 10px; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }"
        )
        srv_layout = QVBoxLayout(self.service_panel)
        srv_layout.setSpacing(6)

        com_layout = QHBoxLayout()
        lbl_com = QLabel("COM-порт:")
        lbl_com.setStyleSheet("font-size: 13px; font-weight: bold;")
        self.combo_com = QComboBox()
        self.combo_com.setEditable(True)
        self.combo_com.setStyleSheet("font-size: 13px; padding: 2px;")

        self.btn_refresh_ports = QPushButton("Обновить")
        self.btn_refresh_ports.setStyleSheet("font-size: 12px; padding: 4px 8px;")
        self.btn_refresh_ports.clicked.connect(self.refresh_com_ports)

        self.btn_apply_port = QPushButton("Применить")
        self.btn_apply_port.setStyleSheet(
            "font-size: 12px; font-weight: bold; padding: 4px 10px; background-color: #d1e7dd;"
        )
        self.btn_apply_port.clicked.connect(self.apply_com_port)

        com_layout.addWidget(lbl_com)
        com_layout.addWidget(self.combo_com, stretch=1)
        com_layout.addWidget(self.btn_refresh_ports)
        com_layout.addWidget(self.btn_apply_port)
        srv_layout.addLayout(com_layout)

        # Сетка ручного управления DO1..DO5 (LED + кнопка переключения)
        do_grid = QGridLayout()
        do_grid.setSpacing(5)
        self.do_leds = {}
        self.manual_do_btns = {}

        for idx, (ch, name) in enumerate(self.DO_NAMES.items()):
            cell = QHBoxLayout()
            cell.setContentsMargins(0, 0, 4, 0)
            led = self._create_led()
            btn = QPushButton(name)
            btn.setStyleSheet("font-size: 12px; padding: 4px 6px;")
            btn.clicked.connect(lambda checked, c=ch: self.toggle_manual_do(c))
            cell.addWidget(led)
            cell.addWidget(btn, stretch=1)
            self.do_leds[ch] = led
            self.manual_do_btns[ch] = btn
            do_grid.addLayout(cell, idx // 3, idx % 3)

        self.btn_manual_off_all = QPushButton("Выкл. всё")
        self.btn_manual_off_all.setStyleSheet(
            "font-size: 12px; font-weight: bold; padding: 4px 6px; background-color: #f8d7da;"
        )
        self.btn_manual_off_all.clicked.connect(self.stop_all)
        do_grid.addWidget(self.btn_manual_off_all, 1, 2)

        srv_layout.addLayout(do_grid)

        self.lbl_srv_ai = QLabel("AI5 (I-7017, #025): Ток = 0.000 мА | Давление = 0.00 атм")
        self.lbl_srv_ai.setStyleSheet(
            "background-color: #e9ecef; padding: 4px 6px; font-size: 13px; font-weight: bold; border-radius: 3px;"
        )
        srv_layout.addWidget(self.lbl_srv_ai)

        self.lbl_srv_flow = QLabel("M-7041D (ID 86): Вх.10 (Прям.) = 0 | Вх.11 (Обр.) = 0 | Вх.12 (Педаль) = 0")
        self.lbl_srv_flow.setStyleSheet(
            "background-color: #fff3cd; padding: 4px 6px; font-size: 12px; font-weight: bold; border-radius: 3px;"
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
        self.btn_stop.setStyleSheet("background-color: red; color: white; height: 50px; font-weight: bold; font-size: 26px;")
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
        self.set_buttons_enabled(False)
        self._update_manual_buttons()
        self.refresh_com_ports()
        self.statusBar().showMessage("Инициализация...")

    def init_comm(self):
        if DUMMY_MODE:
            self.port_ok = True
            self.do_ok = self.ai_ok = self.flow_ok = True
            self.set_buttons_enabled(True)
            self._update_manual_buttons()
            self._update_status_bar()
            self.log("Запущено в тестовом режиме (DUMMY_MODE = True).")
            return

        self.worker = CommWorker(
            port=self.COM_PORT,
            baudrate=self.BAUDRATE,
            addr_do=self.ADDR_DO,
            addr_ao=self.ADDR_AO,
            ai_ch=self.AI_PRESSURE_CH,
            addr_flow=self.ADDR_FLOW,
            flow_reg_start=self.FLOW_FWD_REG,
            flow_fwd_clear_coil=self.FLOW_FWD_CLEAR_COIL,
            flow_rev_clear_coil=self.FLOW_REV_CLEAR_COIL,
            min_ma=self.MIN_MA,
            max_ma=self.MAX_MA,
            min_p=self.MIN_P,
            max_p=self.MAX_P,
            max_missed=self.MAX_MISSED_POLLS
        )
        self.worker.telemetry_updated.connect(self.on_telemetry_updated)
        self.worker.connection_changed.connect(self.on_connection_changed)
        self.worker.slaves_status_changed.connect(self.on_slaves_status_changed)
        self.worker.start()

    def on_telemetry_updated(self, raw_ma, pressure, flow_fwd, flow_rev, pedal_count):
        self.raw_pressure_ma = raw_ma
        self.current_pressure = pressure
        self.pedal_count = pedal_count

        if time.time() >= self.ignore_flow_until:
            self.flow_fwd = flow_fwd
            self.flow_rev = flow_rev

        # Обработка педали (вход 12) с защитой при первом чтении и антидребезгом 1.0 сек
        if self.last_pedal_count is None:
            self.last_pedal_count = pedal_count
        elif pedal_count != self.last_pedal_count:
            self.last_pedal_count = pedal_count
            now = time.time()
            if now >= self.pedal_lock_until:
                self.pedal_lock_until = now + self.PEDAL_DEBOUNCE_TIME
                self.on_pedal_pressed()

    def on_pedal_pressed(self):
        """Реакция на нажатие педали: Старт авто-теста или Стоп"""
        if self.state == "IDLE":
            if self._is_system_ready():
                self.log("[ПЕДАЛЬ] Нажатие -> Запуск Авто-теста")
                self.start_auto_test()
        else:
            self.log("[ПЕДАЛЬ] Нажатие -> Остановка системы")
            self.stop_all()

    def on_connection_changed(self, is_connected, message, is_initial):
        self.port_ok = is_connected
        if is_connected:
            self.log(message)
        else:
            self.do_ok = self.ai_ok = self.flow_ok = False
            self.last_pedal_count = None
            self.log(message, is_error=True)
            if self.state != "IDLE":
                self.log("Текущий тест прерван из-за обрыва связи с COM-портом.", is_error=True)
                self.state = "IDLE"
                self.is_auto_mode = False
                self.auto_pause_start_time = None
            for ch in self.do_states:
                self.do_states[ch] = False
                self.update_valve_ui(ch, False)
            self.set_buttons_enabled(False)
        self._update_manual_buttons()
        self._update_status_bar()

    def on_slaves_status_changed(self, do_ok, ai_ok, flow_ok):
        prev_ready = self._is_system_ready()
        self.do_ok = do_ok
        self.ai_ok = ai_ok
        self.flow_ok = flow_ok
        self._update_status_bar()

        now_ready = self._is_system_ready()

        if now_ready and not prev_ready:
            self.log("Все модули (I-7050D, I-7017, M-7041D) на связи. Стенд готов к работе!", is_success=True)
            if self.state == "IDLE":
                self.set_buttons_enabled(True)
        elif not now_ready and self.port_ok:
            missing = []
            if not do_ok:
                missing.append("I-7050D (Реле)")
            if not ai_ok:
                missing.append("I-7017 (Давление)")
            if not flow_ok:
                missing.append("M-7041D (Расход)")
            missing_str = ", ".join(missing)

            if self.state != "IDLE":
                self.log(f"АВАРИЯ: Потеряна связь с модулями: {missing_str}! Тест остановлен.", is_error=True)
                self.stop_all()
            elif prev_ready:
                self.log(f"Потеряна связь с модулями: {missing_str}. Запуск тестов заблокирован.", is_error=True)

            self.set_buttons_enabled(False)

    def refresh_com_ports(self):
        available = [p.device for p in list_ports.comports()]
        self.combo_com.clear()
        if self.COM_PORT and self.COM_PORT not in available:
            self.combo_com.addItem(self.COM_PORT)
        self.combo_com.addItems(available)
        self.combo_com.setCurrentText(self.COM_PORT)

    def apply_com_port(self):
        new_port = self.combo_com.currentText().strip()
        if not new_port:
            return

        if self.state != "IDLE":
            self.stop_all()

        self.COM_PORT = new_port
        self.settings.setValue("com_port", self.COM_PORT)
        self.settings.sync()

        self.log(f"Сохранен новый COM-порт: {self.COM_PORT}. Переподключение...")
        if self.worker is not None:
            self.port_ok = False
            self.do_ok = self.ai_ok = self.flow_ok = False
            self.last_pedal_count = None
            self.set_buttons_enabled(False)
            self._update_manual_buttons()
            self._update_status_bar()
            self.worker.enqueue_change_port(self.COM_PORT)

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
                self.refresh_com_ports()
                self._update_manual_buttons()
                self.service_panel.setVisible(True)
                self.btn_service.setText("Выйти из сервиса")
                self.log("Сервисный режим активирован.")
            elif ok:
                self.log("Неверный пароль сервисного режима!", is_error=True)

    def toggle_manual_do(self, channel: int):
        """Ручное переключение клапана или насоса из сервисного меню"""
        if self.state != "IDLE" or not self.port_ok:
            return
        new_state = not self.do_states[channel]
        self.write_do(channel, new_state)
        name = self.DO_NAMES.get(channel, f"DO{channel}")
        self.log(f"[СЕРВИС] {name} -> {'ВКЛ' if new_state else 'ВЫКЛ'}")

    def clear_log(self):
        self.log_console.clear()

    def set_buttons_enabled(self, is_enabled):
        self.btn_auto.setEnabled(is_enabled)
        self.btn_fwd.setEnabled(is_enabled)
        self.btn_rev.setEnabled(is_enabled)
        alpha = "255" if is_enabled else "100"
        self.btn_auto.setStyleSheet(f"background-color: rgba(255, 215, 0, {alpha}); height: 50px; font-weight: bold; font-size: 24px;")
        self.btn_fwd.setStyleSheet(f"background-color: rgba(173, 216, 230, {alpha}); height: 50px; font-weight: bold; font-size: 24px;")
        self.btn_rev.setStyleSheet(f"background-color: rgba(144, 238, 144, {alpha}); height: 50px; font-weight: bold; font-size: 24px;")

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

    def log(self, text, is_error=False, is_success=False):
        t = time.strftime("%H:%M:%S")
        if is_error:
            html = f'<span style="color: #d90000; font-weight: bold;">[{t}] {text}</span>'
        elif is_success:
            html = f'<span style="color: #008000; font-weight: bold;">[{t}] {text}</span>'
        else:
            html = f'<span>[{t}] {text}</span>'
        self.log_console.append(html)

    # ================== ОТПРАВКА КОМАНД В ФОНОВЫЙ ПОТОК ==================
    def write_do(self, channel, state):
        self.do_states[channel] = state
        self.update_valve_ui(channel, state)
        if not DUMMY_MODE and self.worker is not None and self.port_ok:
            self.worker.enqueue_do(channel, state)

    def clear_flow_counter(self, is_reverse=False):
        """Сброс нужного счетчика: Coil 522 (вход 10, прямой) или Coil 523 (вход 11, обратный)"""
        if is_reverse:
            self.flow_rev = 0
            coil = self.FLOW_REV_CLEAR_COIL
        else:
            self.flow_fwd = 0
            coil = self.FLOW_FWD_CLEAR_COIL

        self.ignore_flow_until = time.time() + 0.4
        if not DUMMY_MODE and self.worker is not None and self.port_ok:
            self.worker.enqueue_clear_flow(coil)

    # ================== БИЗНЕС ЛОГИКА ==================
    def start_auto_test(self):
        if not self._is_system_ready():
            return
        self.is_auto_mode = True
        self.log("=== СТАРТ: АВТОМАТИЧЕСКИЙ ТЕСТ (Прямой -> Обратный) ===")
        self.start_forward_test(keep_auto=True)

    def start_forward_test(self, keep_auto=False):
        if not self._is_system_ready():
            return
        if not keep_auto:
            self.is_auto_mode = False
        self.set_buttons_enabled(False)
        self.log("--- СТАРТ: Прямой тест (датчик вход 10) ---")
        self.time_data, self.pressure_data = [], []
        self.start_time = time.time()

        self.clear_flow_counter(is_reverse=False)

        self.write_do(self.DO_VALVE_FWD_IN, True)
        self.write_do(self.DO_VALVE_FWD_OUT, True)
        self.write_do(self.DO_VALVE_REV_IN, False)
        self.write_do(self.DO_VALVE_REV_OUT, False)

        QTimer.singleShot(500, lambda: self.write_do(self.DO_PUMP_START, True))
        self.state = "FWD_WAIT_OPEN"
        self._update_manual_buttons()

    def start_reverse_test(self, keep_auto=False):
        if not self._is_system_ready():
            return
        if not keep_auto:
            self.is_auto_mode = False
        self.set_buttons_enabled(False)
        self.log(
            f"--- СТАРТ: Обратный тест (датчик вход 11, стабилизация {int(self.REV_STABILIZE_TIME)} сек + "
            f"контроль {int(self.REV_TEST_TIME)} сек) ---"
        )
        self.time_data, self.pressure_data = [], []
        self.start_time = time.time()
        self.rev_flow_baseline = 0

        self.clear_flow_counter(is_reverse=True)

        self.write_do(self.DO_VALVE_FWD_IN, False)
        self.write_do(self.DO_VALVE_FWD_OUT, False)
        self.write_do(self.DO_VALVE_REV_IN, True)
        self.write_do(self.DO_VALVE_REV_OUT, True)

        QTimer.singleShot(500, lambda: self.write_do(self.DO_PUMP_START, True))
        self.state = "REV_STABILIZE"
        self._update_manual_buttons()

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
        self.auto_pause_start_time = None
        self.set_buttons_enabled(self._is_system_ready())
        self._update_manual_buttons()

    # ================== ГЛАВНЫЙ ЦИКЛ UI ==================
    def main_loop(self):
        if DUMMY_MODE:
            if self.state == "FWD_WAIT_OPEN":
                self.current_pressure += 0.08
                if self.current_pressure > 1.5:
                    self.flow_fwd += 5
            elif self.state == "FWD_WAIT_CLOSE":
                self.current_pressure = max(0.0, self.current_pressure - 0.08)
                if self.current_pressure >= 0.6:
                    self.flow_fwd += 2
            elif self.state in ["REV_STABILIZE", "REV_TEST"]:
                self.current_pressure = min(2.5, self.current_pressure + 0.05)
                if self.state == "REV_STABILIZE" and self.current_pressure < 1.0:
                    self.flow_rev = 2
            else:
                self.current_pressure = max(0.0, self.current_pressure - 0.1)
            self.raw_pressure_ma = self.MIN_MA + (self.current_pressure / self.MAX_P) * (self.MAX_MA - self.MIN_MA)

        if not self.port_ok:
            return

        self.lbl_pump_press.setText(f"{self.current_pressure:.2f}")

        if self.service_mode:
            self.lbl_srv_ai.setText(
                f"AI5 (I-7017, #025): Ток = {self.raw_pressure_ma:.3f} мА | Давление = {self.current_pressure:.2f} атм"
            )
            self.lbl_srv_flow.setText(
                f"M-7041D (ID 86): Вх.10 (Прям.) = {self.flow_fwd} | Вх.11 (Обр.) = {self.flow_rev} | Вх.12 (Педаль) = {self.pedal_count}"
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

            if self.flow_fwd > 0:
                self.fixed_open = self.current_pressure
                self.log(f"ОТКРЫТИЕ зафиксировано: {self.fixed_open:.2f} атм")
                self.write_do(self.DO_PUMP_START, False)
                self.last_flow_val = self.flow_fwd
                self.last_flow_time = time.time()
                self.last_flow_pressure = self.current_pressure
                self.state = "FWD_WAIT_CLOSE"
            elif t_elapsed >= 30.0 and self.current_pressure > 0.1:
                self.log(
                    "ОШИБКА: Нет расхода (вход 10) в течение 30 сек при наличии давления! Проверьте исправность расходомера.",
                    is_error=True
                )
                self.stop_all()

        elif self.state == "FWD_WAIT_CLOSE":
            if self.flow_fwd != self.last_flow_val:
                self.last_flow_val = self.flow_fwd
                self.last_flow_time = time.time()
                self.last_flow_pressure = self.current_pressure
            elif time.time() - self.last_flow_time >= self.FLOW_STOP_TIMEOUT:
                closing_pressure = self.last_flow_pressure
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
                        self.auto_pause_start_time = None
                        self.state = "AUTO_PAUSE"
                        self.log("Ожидание падения давления ниже 0.2 атм перед обратным тестом...")
                    else:
                        self.stop_all()
                else:
                    self.log("РЕЗУЛЬТАТ ПРЯМОГО ТЕСТА: БРАК! Клапан не прошел критерии.", is_error=True)
                    if self.is_auto_mode:
                        self.log("Автоматический тест прерван из-за провала прямого теста.", is_error=True)
                    self.stop_all()

        elif self.state == "AUTO_PAUSE":
            if self.auto_pause_start_time is None:
                if self.current_pressure < 0.2:
                    self.auto_pause_start_time = time.time()
                    self.log(
                        f"Давление ниже 0.2 атм. Пауза {int(self.AUTO_PAUSE_TIME)} сек "
                        f"перед запуском обратного теста..."
                    )
            else:
                if time.time() - self.auto_pause_start_time >= self.AUTO_PAUSE_TIME:
                    self.start_reverse_test(keep_auto=True)

        elif self.state == "REV_STABILIZE":
            t_elapsed = time.time() - self.start_time
            if t_elapsed >= self.REV_STABILIZE_TIME:
                self.rev_flow_baseline = self.flow_rev
                self.rev_test_start_time = time.time()
                self.log(
                    f"Стабилизация завершена (база счетчика вход 11: {self.rev_flow_baseline} ед.). "
                    f"Начат контроль герметичности ({int(self.REV_TEST_TIME)} сек)..."
                )
                self.state = "REV_TEST"

        elif self.state == "REV_TEST":
            t_test = time.time() - self.rev_test_start_time
            delta_flow = self.flow_rev - self.rev_flow_baseline

            if delta_flow > 0:
                self.log(
                    f"РЕЗУЛЬТАТ ОБРАТНОГО ТЕСТА: БРАК! Протечка клапана (+{delta_flow} ед. по входу 11) "
                    f"на {t_test:.1f} сек контроля ({self.current_pressure:.2f} атм).",
                    is_error=True
                )
                self.stop_all()

            elif t_test >= self.REV_TEST_TIME:
                if self.current_pressure >= 2.0:
                    self.log(
                        f"РЕЗУЛЬТАТ ОБРАТНОГО ТЕСТА: УСПЕХ! Клапан герметичен ({self.current_pressure:.2f} атм).",
                        is_success=True
                    )
                else:
                    self.log(
                        f"ТЕСТ ПРОВАЛЕН: Недостаточное давление ({self.current_pressure:.2f} атм). "
                        f"Проведите ревизию насоса и системы!",
                        is_error=True
                    )
                self.stop_all()

    def closeEvent(self, event):
        if self.worker is not None:
            self.worker.stop()
            self.worker.wait(1000)
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ValveTestBench()
    window.show()
    sys.exit(app.exec())