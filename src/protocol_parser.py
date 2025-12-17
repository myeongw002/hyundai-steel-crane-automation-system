# protocol_parser.py
from hyundai_steel_crane_automation_system.msg import HeadCR, ACWL0001Body, WLAC0001Body
from hyundai_steel_crane_automation_system.srv import ACWL0001, WLAC0001
import time
from builtin_interfaces.msg import Time

# 오프셋(바이트 인덱스)
# header: 0:8 msg_id, 8:18 date(10), 18:26 time(8), 26:27 form(1), 27:31 msg_len(4)
MSG_ID_SLICE = slice(0, 8)
DATE_SLICE   = slice(8, 18)
TIME_SLICE   = slice(18, 26)
FORM_SLICE   = slice(26, 27)
MSG_LEN_SLICE= slice(27, 31)

# ACWL0001 body slices (absolute)
ACWL_EQP_CD      = slice(40, 46)   # 6
ACWL_DX_DIRECT   = slice(46, 47)   # 1
ACWL_DY_DIRECT   = slice(47, 48)   # 1
ACWL_DZ          = slice(48, 53)   # 5
ACWL_CR_OP_ID    = slice(53, 71)   # 18
ACWL_PLATE_L     = slice(71, 75)   # 4 (plate_top_len)
ACWL_PLATE_W     = slice(75, 79)   # 4 (plate_max_width)
ACWL_LIDAR_USE   = slice(79, 80)   # 1
ACWL_CAMERA_USE  = slice(80, 81)   # 1

# WLAC0001 body slices (absolute)
WLAC_EQP_CD      = slice(31, 37)
WLAC_RESULT_CODE = slice(37, 41)
WLAC_LEN_RESULT  = slice(41, 45)
WLAC_WIDTH_RESULT = slice(45, 50)

def _ascii_strip(b: bytes) -> str:
    return b.decode('ascii', errors='ignore').rstrip()

def _ascii_int(b: bytes, default=0) -> int:
    s = b.decode('ascii', errors='ignore').strip()
    if not s:
        return default
    try:
        return int(s)
    except ValueError:
        # 허용되는 형식으로 전처리 (+/- 포함)
        try:
            return int(s.replace('+', ''))
        except Exception:
            return default

def _get_timestamp() -> Time:
    """현재 시간을 ROS2 Time 메시지로 반환"""
    now = time.time()
    stamp = Time()
    stamp.sec = int(now)
    stamp.nanosec = int((now % 1) * 1e9)
    return stamp

class ProtocolParser:
    """소켓 데이터 ↔ ROS2 메시지 변환 (fixed-field ASCII)"""

    @staticmethod
    def parse_header(data: bytes) -> HeadCR:
        h = HeadCR()
        h.msg_id = _ascii_strip(data[MSG_ID_SLICE])
        h.date   = _ascii_strip(data[DATE_SLICE])
        h.time   = _ascii_strip(data[TIME_SLICE])
        h.form   = _ascii_strip(data[FORM_SLICE])
        # msg_len is numeric ASCII of 4 chars
        try:
            h.msg_len = int(data[MSG_LEN_SLICE].decode('ascii', errors='ignore'))
        except Exception:
            h.msg_len = 0
        h.filler = ""  # 필요하면 추가로 추출
        return h

    @staticmethod
    def parse_acwl0001(data: bytes) -> ACWL0001.Request:
        req = ACWL0001.Request()
        req.head = ProtocolParser.parse_header(data)

        b = ACWL0001Body()
        b.eqp_cd = _ascii_strip(data[ACWL_EQP_CD])
        # dx_direct, dy_direct는 1바이트 텍스트 필드 ("0" 또는 "1")
        dx_txt = _ascii_strip(data[ACWL_DX_DIRECT])
        b.dx_direct = int(dx_txt) if dx_txt and dx_txt.isdigit() else 0
        dy_txt = _ascii_strip(data[ACWL_DY_DIRECT])
        b.dy_direct = int(dy_txt) if dy_txt and dy_txt.isdigit() else 0
        b.dz_mm = _ascii_int(data[ACWL_DZ], 0)
        b.cr_op_indi_id = _ascii_strip(data[ACWL_CR_OP_ID])
        b.plate_top_len = _ascii_int(data[ACWL_PLATE_L], 0)
        b.plate_max_width = _ascii_int(data[ACWL_PLATE_W], 0)
        b.lidar_use = (_ascii_strip(data[ACWL_LIDAR_USE]) == "1")
        b.camera_use = (_ascii_strip(data[ACWL_CAMERA_USE]) == "1")
        b.raw = data.decode('ascii', errors='ignore')
        b.stamp = _get_timestamp()  # 타임스탬프 기록
        req.body = b
        return req

    @staticmethod
    def parse_wlac0001(data: bytes) -> WLAC0001.Request:
        req = WLAC0001.Request()
        req.head = ProtocolParser.parse_header(data)

        b = WLAC0001Body()
        b.eqp_cd = _ascii_strip(data[WLAC_EQP_CD])
        b.result_code = _ascii_strip(data[WLAC_RESULT_CODE])
        b.len_result_mm = _ascii_int(data[WLAC_LEN_RESULT], 0)
        b.width_result_mm = _ascii_int(data[WLAC_WIDTH_RESULT], 0)
        req.body = b
        return req

    @staticmethod
    def build_wlac0001(request: WLAC0001.Request) -> bytes:
        """WLAC0001 Request -> ASCII bytes ready to send to PLC.
           This function pads/zeros fields to match spec.
        """
        head = request.head
        body = request.body

        # build header
        msg = bytearray()
        msg.extend(head.msg_id.ljust(8).encode('ascii'))
        msg.extend(head.date.ljust(10).encode('ascii'))
        msg.extend(head.time.ljust(8).encode('ascii'))
        msg.extend(head.form.ljust(1).encode('ascii'))

        # placeholder for length (4 bytes), will fill later
        msg_len_index = len(msg)
        msg.extend(b"0000")  # will overwrite

        # filler (9 bytes)
        msg.extend(b" " * 9)

        # body
        msg.extend(body.eqp_cd.ljust(6).encode('ascii'))
        msg.extend(body.result_code.ljust(4).encode('ascii'))
        
        # length result: 부호(+/-) + 4자리 숫자 = 총 5자리
        # ex: +0029, -0100
        l = body.len_result_mm
        if isinstance(l, int):
            ltxt = f"{l:+05d}"  # 부호 포함 5자리 (ex: +0029, -0100)
        else:
            ltxt = str(l).rjust(5)
        msg.extend(ltxt.encode('ascii'))
        
        # width result: 부호(+/-) + 4자리 숫자 = 총 5자리
        # ex: +0029, -0100
        w = body.width_result_mm
        if isinstance(w, int):
            wtxt = f"{w:+05d}"  # 부호 포함 5자리 (ex: +0029, -0100)
        else:
            wtxt = str(w).rjust(5)
        msg.extend(wtxt.encode('ascii'))

        # finalize length
        total_len = len(msg)
        # write length into the reserved 4-byte spot
        msg[msg_len_index:msg_len_index+4] = f"{total_len:04d}".encode('ascii')
        return bytes(msg)
