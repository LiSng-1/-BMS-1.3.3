from saleae.analyzers import HighLevelAnalyzer, AnalyzerFrame
import struct

# 星恒BMS一线通协议A1.3.3 核心常量
SYNC_T1_MIN = 10.0    # 同步信号T1最小脉宽(ms)
SYNC_T2_FIX = 1.0     # 同步信号T2固定脉宽(ms)
STOP_N_MIN = 50.0     # 停止信号N最小脉宽(ms)
BIT_CYCLE = 2.0       # 数据位固定周期(ms)
BIT1_T1 = 0.5         # 逻辑1的T1脉宽(ms)
BIT0_T1 = 1.5         # 逻辑0的T1脉宽(ms)
PULSE_TOLERANCE = 0.5 # 脉宽容差(ms)，适配硬件实际波动

# 报文ID定义
PUBLIC_FRAME_ID = 0x01    # 公有报文(无锡团标)
VOLTAGE_FRAME_ID = 0x3B   # 单串电压报文(北京团标)
SN_FRAME_ID = 0x3C        # 电池唯一码报文(北京团标)
PUBLIC_FRAME_LEN = 20     # 公有报文固定长度(字节)

# 电芯材料映射表(协议表A.3，A1.3.3新增钠电芯)
CELL_MATERIAL_MAP = {
    0x00: "预留", 0x01: "磷酸铁锂", 0x02: "锰酸锂", 0x03: "三元锂",
    0x04: "钴酸锂", 0x05: "聚合锂", 0x06: "钛酸锂", 0x07: "铅酸",
    0x08: "镍氢", 0x09: "钠", 0x0A: "预留", 0x0B~0xFF: "预留"
}

# 电池工作状态映射表(协议表A.2)
BATTERY_WORK_STATE = {
    0x00: "单独放电", 0x01: "单独充电", 0x02: "单独回馈",
    0x03: "预留", 0x04: "预留", 0x05: "预留", 0x06: "预留", 0x07: "预留",
    0x08: "预留", 0x09: "预留", 0x0A: "预留", 0x0B: "预留",
    0x0C: "预留", 0x0D: "预留", 0x0E: "预留", 0x0F: "预留"
}

# 故障码/预警码映射表(协议表A.4，含★/▲标识)
FAULT_CODE_MAP = {
    0x00: "无故障", 0x01: "DOC2P 放电过流二级保护", 0x02: "DOC1P 放电过流一级保护▲",
    0x03: "CUTP 低温充电保护", 0x04: "COTP 充电高温保护★", 0x05: "DOTP 放电高温保护★",
    0x06: "UVP 欠压保护", 0x07: "OVP 过压保护▲", 0x08: "COCP 充电过流保护",
    0x09: "DUTP 放电低温保护", 0x0A: "CMOSP 充电MOS故障", 0x0B: "DMOSP 放电MOS故障",
    0x0C~0x0F: "预留"
}

# BMS当前状态位解析(协议表A.5 Bit定义)
BMS_STATE_BIT = {
    0: "整车是否合法(0=合法,1=不合法)",
    2: "充电器连接状态DETC(0=未连接,1=已连接)",
    3: "整车是否合法(0=合法,1=不合法)",
    5: "预放电MOS状态(0=关,1=开)",
    6: "放电MOS状态(0=关,1=开)",
    7: "充电MOS状态(0=关,1=开)"
}

class XingHengBmsSingleWire(HighLevelAnalyzer):
    """星恒BMS一线通协议高电平解析器(Saleae Logic 2)"""
    # 定义Saleae表格输出列
    result_types = {
        "frame": {"format": "星恒一线通 - {frame_type}"},
        "public_field": {"format": "公有报文 - {field}: {value}"},
        "private_frame": {"format": "私有报文 - {id}: {desc}"},
        "checksum": {"format": "校验 - {result}: 计算{calc} | 接收{recv}"},
        "error": {"format": "解析错误 - {reason}"}
    }

    def __init__(self):
        # 状态机：IDLE(空闲)→SYNC_T1(同步T1)→SYNC_T2(同步T2)→DATA(接收数据)→STOP(停止信号)
        self.state = "IDLE"
        self.samplerate = None  # 采样率(由Saleae自动传入)
        self.frame_start = None # 帧起始时间
        self.data_buffer = []   # 报文字节缓冲区
        self.bit_buffer = []    # 数据位缓冲区(8位拼接1字节)
        self.pulse_dur = 0.0    # 脉冲持续时间(ms)

    def decode(self, frame: AnalyzerFrame):
        """核心解码方法，Saleae逐帧调用"""
        # 初始化采样率
        if self.samplerate is None:
            self.samplerate = frame.samplerate
        # 计算脉冲持续时间(ms)：duration(秒) * 1000
        self.pulse_dur = frame.duration * 1000

        # 状态机驱动解析
        if self.state == "IDLE":
            return self._idle_state(frame)
        elif self.state == "SYNC_T1":
            return self._sync_t1_state(frame)
        elif self.state == "SYNC_T2":
            return self._sync_t2_state(frame)
        elif self.state == "DATA":
            return self._data_state(frame)
        elif self.state == "STOP":
            return self._stop_state(frame)
        return None

    def _idle_state(self, frame):
        """空闲状态：检测上升沿，进入同步T1检测"""
        if frame.type == "rising_edge":
            self.frame_start = frame.start_time
            self.state = "SYNC_T1"
        return None

    def _sync_t1_state(self, frame):
        """同步T1检测：T1≥10ms，下降沿进入T2"""
        if frame.type == "falling_edge":
            if self.pulse_dur >= SYNC_T1_MIN - PULSE_TOLERANCE:
                self.state = "SYNC_T2"
            else:
                self.state = "IDLE"
                return AnalyzerFrame("error", self.frame_start, frame.end_time,
                    {"reason": f"同步T1脉宽无效: {self.pulse_dur:.1f}ms(要求≥10ms)"})
        return None

    def _sync_t2_state(self, frame):
        """同步T2检测：T2=1ms±0.5ms，上升沿进入数据接收"""
        if frame.type == "rising_edge":
            if abs(self.pulse_dur - SYNC_T2_FIX) <= PULSE_TOLERANCE:
                self.bit_buffer = []
                self.data_buffer = []
                self.state = "DATA"
            else:
                self.state = "IDLE"
                return AnalyzerFrame("error", self.frame_start, frame.end_time,
                    {"reason": f"同步T2脉宽无效: {self.pulse_dur:.1f}ms(要求1ms)"})
        return None

    def _data_state(self, frame):
        """数据接收：解析逻辑0/1，8位拼接字节，检测停止信号"""
        if frame.type == "falling_edge":
            # 解析单bit：根据T1脉宽判断0/1
            if abs(self.pulse_dur - BIT1_T1) <= PULSE_TOLERANCE:
                self.bit_buffer.append(1)
            elif abs(self.pulse_dur - BIT0_T1) <= PULSE_TOLERANCE:
                self.bit_buffer.append(0)
            else:
                return AnalyzerFrame("error", frame.start_time, frame.end_time,
                    {"reason": f"数据位脉宽无效: {self.pulse_dur:.1f}ms(0=1.5ms/1=0.5ms)"})
            
            # 8位拼接1字节(先传最低位，协议6.1.1.1)
            if len(self.bit_buffer) == 8:
                byte_val = 0
                for i, bit in enumerate(self.bit_buffer):
                    byte_val += bit << i
                self.data_buffer.append(byte_val)
                self.bit_buffer = []

                # 公有报文固定20字节，触发停止信号检测
                if len(self.data_buffer) == PUBLIC_FRAME_LEN:
                    self.state = "STOP"

        # 检测停止信号：高电平脉宽≥50ms，直接进入停止状态
        elif frame.type == "rising_edge" and self.pulse_dur >= STOP_N_MIN - PULSE_TOLERANCE:
            self.state = "STOP"
        return None

    def _stop_state(self, frame):
        """停止信号检测：解析缓冲区报文，重置状态机"""
        if self.pulse_dur >= STOP_N_MIN - PULSE_TOLERANCE and len(self.data_buffer) > 0:
            parse_result = self._parse_frame(self.data_buffer)
            self.state = "IDLE"
            self.data_buffer = []
            return parse_result
        return None

    def _parse_frame(self, data):
        """报文解析：区分公有/单串电压/唯一码报文"""
        frame_id = data[0]
        # 解析公有报文(0x01，固定20字节)
        if frame_id == PUBLIC_FRAME_ID and len(data) == PUBLIC_FRAME_LEN:
            return self._parse_public_frame(data)
        # 解析单串电压报文(0x3B，长度可变)
        elif frame_id == VOLTAGE_FRAME_ID:
            return AnalyzerFrame("private_frame", self.frame_start, None, {
                "id": "0x3B",
                "desc": f"单串电压报文 | 长度:{len(data)}字节 | 发送周期60s"
            })
        # 解析电池唯一码报文(0x3C，长度可变)
        elif frame_id == SN_FRAME_ID:
            return AnalyzerFrame("private_frame", self.frame_start, None, {
                "id": "0x3C",
                "desc": f"电池唯一码报文 | 长度:{len(data)}字节 | 发送周期60s(DETD接入先发数帧)"
            })
        # 未知私有报文(主机厂自定义ID)
        else:
            return AnalyzerFrame("private_frame", self.frame_start, None, {
                "id": hex(frame_id),
                "desc": f"未知私有报文 | 长度:{len(data)}字节 | 需主机厂自定义解析"
            })

    def _parse_public_frame(self, data):
        """解析公有报文(协议表A.2)，含校验、物理值换算"""
        # 1. 和校验验证：序号0-18之和的低8位 = 序号19
        calc_check = sum(data[0:19]) & 0xFF
        recv_check = data[19]
        check_result = "成功" if calc_check == recv_check else "失败"
        # 生成校验结果帧
        check_frame = AnalyzerFrame("checksum", self.frame_start, None, {
            "result": check_result,
            "calc": hex(calc_check),
            "recv": hex(recv_check)
        })
        if check_result == "失败":
            return check_frame

        # 2. 解析核心字段(按协议公式P=C*R+F换算物理值，小端模式)
        fields = {}
        # 基础信息
        fields["电池厂商"] = "星恒(0x01)" if data[2] == 0x01 else "无效(0xFF)" if data[2] == 0xFF else hex(data[2])
        fields["电池型号"] = str(data[3]) if 0 <= data[3] <=15 else "无效(0xFF)"
        fields["电芯材料"] = CELL_MATERIAL_MAP.get(data[4], f"未知(0x{data[4]:02X})")
        # 额定参数
        rated_volt = struct.unpack("<H", bytes(data[5:7]))[0]
        fields["额定电压"] = f"{rated_volt*0.1}V" if rated_volt != 0xFFFF else "无效(0xFFFF)"
        rated_cap = struct.unpack("<H", bytes(data[7:9]))[0]
        fields["额定容量"] = f"{rated_cap*0.1}AH" if rated_cap != 0xFFFF else "无效(0xFFFF)"
        # 实时状态
        soc = data[9]
        fields["剩余电量(SOC)"] = f"{soc*0.5}%" if soc != 0xFF else "无效(0xFF)"
        curr_volt = struct.unpack("<H", bytes(data[10:12]))[0]
        fields["当前工作电压"] = f"{curr_volt*0.1}V" if curr_volt != 0xFFFF else "无效(0xFFFF)"
        curr_curr = struct.unpack("<H", bytes(data[12:14]))[0]
        fields["当前工作电流"] = f"{(curr_curr-500)*0.1}A" if curr_curr != 0xFFFF else "无效(0xFFFF)"
        # 温度信息(偏移-40℃)
        temp_max = data[14]
        fields["电池最高温度"] = f"{temp_max-40}℃" if temp_max != 0xFF else "无效(0xFF)"
        temp_min = data[15]
        fields["电池最低温度"] = f"{temp_min-40}℃" if temp_min != 0xFF else "无效(0xFF)"
        mos_temp = data[16]
        fields["MOS温度"] = f"{mos_temp-40}℃" if mos_temp != 0xFF else "无效(0xFF)"
        # 故障与状态
        fault_code = data[17]
        fields["电池故障码"] = FAULT_CODE_MAP.get(fault_code, f"未知(0x{fault_code:02X})")
        work_state = data[18]
        fields["电池工作状态"] = BATTERY_WORK_STATE.get(work_state, f"未知(0x{work_state:02X})")

        # 生成主解析帧，详情携带所有字段
        main_field = list(fields.items())[0]
        main_frame = AnalyzerFrame("public_field", self.frame_start, None, {
            "field": main_field[0],
            "value": main_field[1]
        })
        main_frame.data["all_fields"] = fields
        main_frame.data["checksum_result"] = check_result
        return main_frame
