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
WLAC_EQP_CD      = slice(31, 37)   # 6
WLAC_RESULT_CODE = slice(37, 41)   # 4
WLAC_LEN_RESULT_P1_P2  = slice(41, 46)   # 5 (부호 포함)
WLAC_LEN_RESULT_P3_P4  = slice(46, 51)   # 5 (부호 포함)
WLAC_WIDTH_RESULT_P5_P6 = slice(51, 56)  # 5 (부호 포함)
WLAC_WIDTH_RESULT_P7_P8 = slice(56, 61)  # 5 (부호 포함)

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
        b.len_result_p1_2 = _ascii_int(data[WLAC_LEN_RESULT_P1_P2], 0)
        b.len_result_p3_4 = _ascii_int(data[WLAC_LEN_RESULT_P3_P4], 0)
        b.width_result_p5_6 = _ascii_int(data[WLAC_WIDTH_RESULT_P5_P6], 0)
        b.width_result_p7_8 = _ascii_int(data[WLAC_WIDTH_RESULT_P7_P8], 0)
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
        
        # length result P1-P2: 부호 포함 5자리 숫자
        l1 = body.len_result_p1_2
        if isinstance(l1, int):
            l1txt = f"{l1:+05d}"  # 부호 포함 5자리
        else:
            l1txt = str(l1).rjust(5)
        msg.extend(l1txt.encode('ascii'))
        
        # length result P3-P4: 부호 포함 5자리 숫자
        l2 = body.len_result_p3_4
        if isinstance(l2, int):
            l2txt = f"{l2:+05d}"  # 부호 포함 5자리
        else:
            l2txt = str(l2).rjust(5)
        msg.extend(l2txt.encode('ascii'))
        
        # width result P5-P6: 부호 포함 5자리 숫자
        w1 = body.width_result_p5_6
        if isinstance(w1, int):
            w1txt = f"{w1:+05d}"  # 부호 포함 5자리
        else:
            w1txt = str(w1).rjust(5)
        msg.extend(w1txt.encode('ascii'))
        
        # width result P7-P8: 부호 포함 5자리 숫자
        w2 = body.width_result_p7_8
        if isinstance(w2, int):
            w2txt = f"{w2:+05d}"  # 부호 포함 5자리
        else:
            w2txt = str(w2).rjust(5)
        msg.extend(w2txt.encode('ascii'))

        # finalize length
        total_len = len(msg)
        # write length into the reserved 4-byte spot
        msg[msg_len_index:msg_len_index+4] = f"{total_len:04d}".encode('ascii')
        return bytes(msg)


def send_result_to_plc(wlac_client, result: dict, original_request, 
                       logger=None, timeout_sec: float = 10.0):
    """
    PLC로 측정 결과 전송 (WLAC0001)
    
    Args:
        wlac_client: ROS2 WLAC0001 서비스 클라이언트
        result: 측정 결과 딕셔너리 (result_code, len_p1_2, len_p3_4, width_p5_6, width_p7_8)
        original_request: 원본 ACWL0001 요청 (eqp_cd 추출용)
        logger: 로거 (옵션)
        timeout_sec: 타임아웃 시간 (기본 10초)
    
    Returns:
        (wlac_request_dict, wlac_response_dict) 튜플
    
    Raises:
        Exception: WLAC0001 서비스가 없거나 PLC가 거부한 경우
        TimeoutError: 응답 타임아웃
    """
    from data_logger import convert_wlac_request_to_dict, convert_wlac_response_to_dict
    
    # WLAC 요청 메시지 생성
    wlac_request = WLAC0001.Request()
    
    wlac_request.head = HeadCR()
    wlac_request.head.msg_id = 'WLAC0001'
    wlac_request.head.date = time.strftime('%Y-%m-%d')
    wlac_request.head.time = time.strftime('%H-%M-%S')
    wlac_request.head.form = 'I'
    wlac_request.head.msg_len = 60
    wlac_request.head.filler = ''
    
    wlac_request.body = WLAC0001Body()
    wlac_request.body.eqp_cd = original_request.body.eqp_cd
    wlac_request.body.result_code = result['result_code']
    wlac_request.body.len_result_p1_2 = result['len_p1_2']
    wlac_request.body.len_result_p3_4 = result['len_p3_4']
    wlac_request.body.width_result_p5_6 = result['width_p5_6']
    wlac_request.body.width_result_p7_8 = result['width_p7_8']
    
    now = time.time()
    wlac_request.body.stamp = Time()
    wlac_request.body.stamp.sec = int(now)
    wlac_request.body.stamp.nanosec = int((now % 1) * 1e9)
    
    # 서비스 대기
    if not wlac_client.wait_for_service(timeout_sec=5.0):
        raise Exception('WLAC0001 service not available')
    
    # 비동기 호출
    future = wlac_client.call_async(wlac_request)
    
    # 응답 대기
    timeout_start = time.time()
    while not future.done():
        if time.time() - timeout_start > timeout_sec:
            raise TimeoutError('WLAC0001 response timeout')
        time.sleep(0.01)
    
    response = future.result()
    
    # 딕셔너리 변환
    wlac_req_dict = convert_wlac_request_to_dict(wlac_request)
    wlac_res_dict = convert_wlac_response_to_dict(response)
    
    # 결과 확인
    if response.stored:
        if logger:
            logger.info('✅ Result sent to PLC')
    else:
        raise Exception(f'PLC rejected: {response.error}')
    
    return wlac_req_dict, wlac_res_dict
