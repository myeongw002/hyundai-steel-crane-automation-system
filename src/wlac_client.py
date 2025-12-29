#!/usr/bin/env python3
"""
WLAC0001 서비스 클라이언트 (테스트용)
InferenceNode → WCS 측정 결과 전송 시뮬레이션
"""

import rclpy
from rclpy.node import Node
import time
import sys
import termios
import tty

from hyundai_steel_crane_automation_system.srv import WLAC0001
from hyundai_steel_crane_automation_system.msg import WLAC0001Body, HeadCR
from builtin_interfaces.msg import Time as TimeMsg


class WLAC0001DemoClient(Node):
    """WLAC0001 서비스 클라이언트 (측정 결과 전송 테스트)"""
    
    # 결과 코드 정의
    RESULT_CODES = {
        '0000': 'SUCCESS - 정상 측정',
        '0010': 'WARNING - 치수 경미한 이탈',
        '0020': 'ERROR - 치수 허용 범위 초과',
        '0030': 'FAIL - 측정 실패'
    }
    
    def __init__(self):
        super().__init__('wlac0001_demo_client')
        
        # WLAC0001 클라이언트 생성
        self.wlac_client = self.create_client(
            WLAC0001,
            '/wlac0001_service'
        )
        
        # 시나리오 카운터
        self.scenario_count = 0
        
        self.get_logger().info('🚀 WLAC0001 Demo Client initialized')
    
    def wait_for_service(self, timeout_sec=10.0):
        """서비스 대기"""
        self.get_logger().info(f'Waiting for WLAC0001 service...')
        
        if not self.wlac_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('❌ WLAC0001 service not available')
            return False
        
        self.get_logger().info('✅ WLAC0001 service ready')
        return True
    
    def send_request(
        self,
        eqp_cd: str = 'CRANE_V01',
        result_code: str = '0000',
        len_result_p1_2: int = 0,
        len_result_p3_4: int = 0,
        width_result_p5_6: int = 0,
        width_result_p7_8: int = 0
    ):
        """
        WLAC0001 요청 전송
        
        Args:
            eqp_cd: 설비 코드 (CRANE_V01, CRANE_V02, CRANE_TEST)
            result_code: 결과 코드 ("0000", "0010", "0020", "0030")
            len_result_p1_2: 길이 P1-P2 차이값 (mm)
            len_result_p3_4: 길이 P3-P4 차이값 (mm)
            width_result_p5_6: 폭 P5-P6 차이값 (mm)
            width_result_p7_8: 폭 P7-P8 차이값 (mm)
        """
        self.scenario_count += 1
        
        # 요청 메시지 생성
        request = WLAC0001.Request()
        
        # ========== head 설정 ==========
        request.head = HeadCR()
        request.head.msg_id = 'WLAC0001'
        request.head.date = time.strftime('%Y-%m-%d')
        request.head.time = time.strftime('%H:%M:%S')
        request.head.form = '0'
        request.head.msg_len = 64  # 바디 크기
        request.head.filler = ''
        
        # ========== Body 설정 ==========
        request.body = WLAC0001Body()
        request.body.eqp_cd = eqp_cd
        request.body.result_code = result_code
        request.body.len_result_p1_2 = len_result_p1_2
        request.body.len_result_p3_4 = len_result_p3_4
        request.body.width_result_p5_6 = width_result_p5_6
        request.body.width_result_p7_8 = width_result_p7_8
        
        # Timestamp
        now = time.time()
        request.body.stamp = TimeMsg()
        request.body.stamp.sec = int(now)
        request.body.stamp.nanosec = int((now % 1) * 1e9)
        
        # 요청 정보 출력
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info(f'📤 Sending WLAC0001 Request #{self.scenario_count}:')
        self.get_logger().info(f'  Equipment        : {eqp_cd}')
        self.get_logger().info(f'  Result Code      : {result_code} - {self.RESULT_CODES.get(result_code, "UNKNOWN")}')
        self.get_logger().info(f'  Length P1-P2 Diff: {len_result_p1_2:+5d} mm')
        self.get_logger().info(f'  Length P3-P4 Diff: {len_result_p3_4:+5d} mm')
        self.get_logger().info(f'  Width P5-P6 Diff : {width_result_p5_6:+5d} mm')
        self.get_logger().info(f'  Width P7-P8 Diff : {width_result_p7_8:+5d} mm')
        self.get_logger().info(f'  Timestamp        : {request.head.date} {request.head.time}')
        self.get_logger().info('=' * 70)
        
        # 비동기 호출
        future = self.wlac_client.call_async(request)
        
        return future
    
    def handle_response(self, future):
        """응답 처리"""
        try:
            response = future.result()
            
            self.get_logger().info('=' * 70)
            self.get_logger().info('📥 WLAC0001 Response Received:')
            
            if response.stored:
                self.get_logger().info('  ✅ Status: STORED')
                self.get_logger().info('  📊 Measurement data successfully stored in WCS.')
            else:
                self.get_logger().warning('  ❌ Status: REJECTED')
                self.get_logger().warning(f'  ⚠️  Error: {response.error}')
            
            self.get_logger().info('=' * 70 + '\n')
            
            return response.stored
            
        except Exception as e:
            self.get_logger().error(f'❌ Service call failed: {e}')
            return False


def get_key():
    """키보드 입력 받기 (non-blocking)"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def print_menu():
    """메뉴 출력"""
    print("\n" + "=" * 70)
    print("🎮 WLAC0001 Demo - Measurement Result Test Client")
    print("=" * 70)
    print("📋 시나리오 선택:")
    print()
    print("  [1] 정상 측정 (SUCCESS - 0000)")
    print("      - 모든 치수가 허용 범위 내")
    print("      - 차이값: ±10mm 이내")
    print()
    print("  [2] 경미한 이탈 (WARNING - 0010)")
    print("      - 일부 치수가 약간 벗어남")
    print("      - 차이값: ±30mm 정도")
    print()
    print("  [3] 허용 범위 초과 (ERROR - 0020)")
    print("      - 치수가 허용 범위 초과")
    print("      - 차이값: ±100mm 이상")
    print()
    print("  [4] 측정 실패 (FAIL - 0030)")
    print("      - 측정 자체가 실패")
    print("      - 차이값: 모두 0")
    print()
    print("  [5] 길이 치수 테스트")
    print("      - P1-P2: +50mm, P3-P4: -45mm")
    print("      - 폭은 정상")
    print()
    print("  [6] 폭 치수 테스트")
    print("      - P5-P6: +35mm, P7-P8: -40mm")
    print("      - 길이는 정상")
    print()
    print("  [7] 비대칭 측정 결과")
    print("      - 한쪽은 크고 한쪽은 작음")
    print("      - 판재가 기울어진 경우")
    print()
    print("  [8] 연속 측정 (5회)")
    print("      - 랜덤한 측정값으로 5회 전송")
    print()
    print("  [9] 잘못된 장비 코드 테스트")
    print("      - 유효하지 않은 eqp_cd로 거부 테스트")
    print()
    print("  [0] 커스텀 입력")
    print("      - 사용자 정의 파라미터")
    print()
    print("  [h] 도움말 (메뉴 재출력)")
    print("  [q] 종료")
    print()
    print("=" * 70)
    print("키를 누르세요: ", end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    
    demo_client = WLAC0001DemoClient()
    
    try:
        # 서비스 대기
        if not demo_client.wait_for_service(timeout_sec=10.0):
            demo_client.get_logger().error('Service timeout. Exiting...')
            return
        
        # 메뉴 출력
        print_menu()
        
        # 키보드 입력 루프
        while rclpy.ok():
            key = get_key()
            print(key)  # 입력된 키 출력
            
            future = None
            
            # ========================================
            # 시나리오 1: 정상 측정 (SUCCESS - 0000)
            # ========================================
            if key == '1':
                demo_client.get_logger().info('\n🎬 [1] 정상 측정 (SUCCESS - 0000)')
                future = demo_client.send_request(
                    eqp_cd='CRANE_V01',
                    result_code='0000',
                    len_result_p1_2=1234,      # +5mm
                    len_result_p3_4=-1234,     # -3mm
                    width_result_p5_6=1234,    # +2mm
                    width_result_p7_8=1234    # -4mm
                )
            
            # ========================================
            # 시나리오 2: 경미한 이탈 (WARNING - 0010)
            # ========================================
            elif key == '2':
                demo_client.get_logger().info('\n🎬 [2] 경미한 이탈 (WARNING - 0010)')
                future = demo_client.send_request(
                    eqp_cd='CRANE_V01',
                    result_code='0010',
                    len_result_p1_2=28,     # +28mm
                    len_result_p3_4=-32,    # -32mm
                    width_result_p5_6=25,   # +25mm
                    width_result_p7_8=-30   # -30mm
                )
            
            # ========================================
            # 시나리오 3: 허용 범위 초과 (ERROR - 0020)
            # ========================================
            elif key == '3':
                demo_client.get_logger().info('\n🎬 [3] 허용 범위 초과 (ERROR - 0020)')
                future = demo_client.send_request(
                    eqp_cd='CRANE_V02',
                    result_code='0020',
                    len_result_p1_2=120,    # +120mm
                    len_result_p3_4=-105,   # -105mm
                    width_result_p5_6=95,   # +95mm
                    width_result_p7_8=-88   # -88mm
                )
            
            # ========================================
            # 시나리오 4: 측정 실패 (FAIL - 0030)
            # ========================================
            elif key == '4':
                demo_client.get_logger().info('\n🎬 [4] 측정 실패 (FAIL - 0030)')
                future = demo_client.send_request(
                    eqp_cd='CRANE_TEST',
                    result_code='0030',
                    len_result_p1_2=0,
                    len_result_p3_4=0,
                    width_result_p5_6=0,
                    width_result_p7_8=0
                )
            
            # ========================================
            # 시나리오 5: 길이 치수 테스트
            # ========================================
            elif key == '5':
                demo_client.get_logger().info('\n🎬 [5] 길이 치수 테스트')
                future = demo_client.send_request(
                    eqp_cd='CRANE_V01',
                    result_code='0010',
                    len_result_p1_2=50,     # +50mm
                    len_result_p3_4=-45,    # -45mm
                    width_result_p5_6=2,    # 정상
                    width_result_p7_8=-3    # 정상
                )
            
            # ========================================
            # 시나리오 6: 폭 치수 테스트
            # ========================================
            elif key == '6':
                demo_client.get_logger().info('\n🎬 [6] 폭 치수 테스트')
                future = demo_client.send_request(
                    eqp_cd='CRANE_V01',
                    result_code='0010',
                    len_result_p1_2=3,      # 정상
                    len_result_p3_4=-2,     # 정상
                    width_result_p5_6=35,   # +35mm
                    width_result_p7_8=-40   # -40mm
                )
            
            # ========================================
            # 시나리오 7: 비대칭 측정 결과
            # ========================================
            elif key == '7':
                demo_client.get_logger().info('\n🎬 [7] 비대칭 측정 결과')
                future = demo_client.send_request(
                    eqp_cd='CRANE_V02',
                    result_code='0010',
                    len_result_p1_2=60,     # +60mm (한쪽 큼)
                    len_result_p3_4=-10,    # -10mm (한쪽 작음)
                    width_result_p5_6=45,   # +45mm
                    width_result_p7_8=-8    # -8mm
                )
            
            # ========================================
            # 시나리오 8: 연속 측정 (5회)
            # ========================================
            elif key == '8':
                demo_client.get_logger().info('\n🎬 [8] 연속 측정 (5회)')
                
                import random
                
                for i in range(5):
                    demo_client.get_logger().info(f'\n🔄 측정 {i+1}/5')
                    
                    # 랜덤 측정값 생성
                    len_p1_p2 = random.randint(-50, 50)
                    len_p3_p4 = random.randint(-50, 50)
                    width_p5_p6 = random.randint(-50, 50)
                    width_p7_p8 = random.randint(-50, 50)
                    
                    # 결과 코드 결정
                    max_abs = max(abs(len_p1_p2), abs(len_p3_p4), 
                                  abs(width_p5_p6), abs(width_p7_p8))
                    
                    if max_abs <= 10:
                        result_code = '0000'  # SUCCESS
                    elif max_abs <= 50:
                        result_code = '0010'  # WARNING
                    else:
                        result_code = '0020'  # ERROR
                    
                    future_batch = demo_client.send_request(
                        eqp_cd='CRANE_V01',
                        result_code=result_code,
                        len_result_p1_2=len_p1_p2,
                        len_result_p3_4=len_p3_p4,
                        width_result_p5_6=width_p5_p6,
                        width_result_p7_8=width_p7_p8
                    )
                    
                    rclpy.spin_until_future_complete(demo_client, future_batch, timeout_sec=2.0)
                    if future_batch.done():
                        demo_client.handle_response(future_batch)
                    
                    if i < 4:
                        time.sleep(0.5)  # 0.5초 대기
                
                future = None  # 이미 처리됨
            
            # ========================================
            # 시나리오 9: 잘못된 장비 코드 테스트
            # ========================================
            elif key == '9':
                demo_client.get_logger().info('\n🎬 [9] 잘못된 장비 코드 테스트')
                future = demo_client.send_request(
                    eqp_cd='INVALID_CRANE',  # 잘못된 코드
                    result_code='0000',
                    len_result_p1_2=5,
                    len_result_p3_4=-3,
                    width_result_p5_6=2,
                    width_result_p7_8=-4
                )
            
            # ========================================
            # 시나리오 0: 커스텀 입력
            # ========================================
            elif key == '0':
                demo_client.get_logger().info('\n🎬 [0] 커스텀 입력')
                
                # 원래 터미널 설정 복원
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                
                print("\n설비 코드 (예: CRANE_V01, 기본=CRANE_V01): ", end='', flush=True)
                eqp_cd = input().strip() or 'CRANE_V01'
                
                print("결과 코드 (0000/0010/0020/0030, 기본=0000): ", end='', flush=True)
                result_code = input().strip() or '0000'
                
                print("길이 P1-P2 차이 (mm, 기본=0): ", end='', flush=True)
                len_p1_p2 = int(input().strip() or '0')
                
                print("길이 P3-P4 차이 (mm, 기본=0): ", end='', flush=True)
                len_p3_p4 = int(input().strip() or '0')
                
                print("폭 P5-P6 차이 (mm, 기본=0): ", end='', flush=True)
                width_p5_p6 = int(input().strip() or '0')
                
                print("폭 P7-P8 차이 (mm, 기본=0): ", end='', flush=True)
                width_p7_p8 = int(input().strip() or '0')
                
                future = demo_client.send_request(
                    eqp_cd=eqp_cd,
                    result_code=result_code,
                    len_result_p1_2=len_p1_p2,
                    len_result_p3_4=len_p3_p4,
                    width_result_p5_6=width_p5_p6,
                    width_result_p7_8=width_p7_p8
                )
            
            # ========================================
            # 도움말
            # ========================================
            elif key == 'h' or key == 'H':
                print_menu()
                continue
            
            # ========================================
            # 종료
            # ========================================
            elif key == 'q' or key == 'Q':
                demo_client.get_logger().info('\n👋 Exiting demo...')
                break
            
            # ========================================
            # 잘못된 입력
            # ========================================
            else:
                print(f"\n⚠️  Unknown key: '{key}' (Press 'h' for help)")
                continue
            
            # 응답 처리 (시나리오 8 제외)
            if future is not None:
                rclpy.spin_until_future_complete(demo_client, future, timeout_sec=5.0)
                
                if future.done():
                    demo_client.handle_response(future)
                else:
                    demo_client.get_logger().error('⏱️  Service timeout')
            
            # 다음 입력 대기
            print("\n키를 누르세요 (h=도움말, q=종료): ", end='', flush=True)
        
        demo_client.get_logger().info('✅ Demo completed!')
        
    except KeyboardInterrupt:
        demo_client.get_logger().info('\n👋 Demo interrupted by user (Ctrl+C)')
    
    except Exception as e:
        demo_client.get_logger().error(f'❌ Error: {e}')
        import traceback
        traceback.print_exc()
    
    finally:
        # 터미널 설정 복원
        try:
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except:
            pass
        
        demo_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
