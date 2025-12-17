#!/usr/bin/env python3
"""
ACWL0001 서비스 요청 데모 클라이언트 (키보드 인터랙티브)
PLC → Crane Automation 작업 지시 시뮬레이션
"""

import rclpy
from rclpy.node import Node
import time
import sys
import termios
import tty

from hyundai_steel_crane_automation_system.srv import ACWL0001
from hyundai_steel_crane_automation_system.msg import ACWL0001Body, HeadCR
from builtin_interfaces.msg import Time as TimeMsg


class ACWL0001DemoClient(Node):
    """ACWL0001 서비스 요청 데모"""
    
    def __init__(self):
        super().__init__('acwl0001_demo_client')
        
        # ACWL0001 클라이언트 생성
        self.acwl_client = self.create_client(
            ACWL0001,
            'acwl0001_service'
        )
        
        # 시나리오 카운터
        self.scenario_count = 0
        
        self.get_logger().info('🚀 ACWL0001 Demo Client initialized')
    
    def wait_for_service(self, timeout_sec=10.0):
        """서비스 대기"""
        self.get_logger().info(f'Waiting for ACWL0001 service...')
        
        if not self.acwl_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('❌ ACWL0001 service not available')
            return False
        
        self.get_logger().info('✅ ACWL0001 service ready')
        return True
    
    def send_request(
        self,
        eqp_cd: str = 'EQP001',
        dx_direct: int = 0,
        dy_direct: int = 0,
        dz_mm: int = 0,
        plate_top_len: int = 2000,
        plate_max_width: int = 1500,
        cr_op_indi_id: str = '',
        camera_use: bool = True,
        lidar_use: bool = True
    ):
        """
        ACWL0001 요청 전송
        
        Args:
            eqp_cd: 설비 코드
            dx_direct: X 방향 (0=정방향, 1=역방향)
            dy_direct: Y 방향 (0=정방향, 1=역방향)
            dz_mm: Z 높이 (mm)
            plate_top_len: 판재 상단 길이 (mm)
            plate_max_width: 판재 최대 폭 (mm)
            cr_op_indi_id: 크레인 작업 지시 ID
            camera_use: 카메라 사용 여부
            lidar_use: LiDAR 사용 여부
        """
        self.scenario_count += 1
        
        # 요청 메시지 생성
        request = ACWL0001.Request()
        
        # ========== head 설정 ==========
        request.head = HeadCR()
        request.head.msg_id = 'ACWL0001'
        request.head.date = time.strftime('%Y-%m-%d')
        request.head.time = time.strftime('%H:%M:%S')
        request.head.form = '0'
        request.head.msg_len = 50
        request.head.filler = ''
        
        # ========== Body 설정 ==========
        request.body = ACWL0001Body()
        request.body.eqp_cd = eqp_cd
        request.body.dx_direct = dx_direct
        request.body.dy_direct = dy_direct
        request.body.dz_mm = dz_mm
        request.body.cr_op_indi_id = cr_op_indi_id
        request.body.plate_top_len = plate_top_len
        request.body.plate_max_width = plate_max_width
        request.body.camera_use = camera_use
        request.body.lidar_use = lidar_use
        request.body.raw = ''
        
        # Timestamp
        now = time.time()
        request.body.stamp = TimeMsg()
        request.body.stamp.sec = int(now)
        request.body.stamp.nanosec = int((now % 1) * 1e9)
        
        # 요청 정보 출력
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info(f'📤 Sending ACWL0001 Request #{self.scenario_count}:')
        self.get_logger().info(f'  Equipment     : {eqp_cd}')
        self.get_logger().info(f'  DX Direction  : {"정방향 (0)" if dx_direct == 0 else "역방향 (1)"}')
        self.get_logger().info(f'  DY Direction  : {"정방향 (0)" if dy_direct == 0 else "역방향 (1)"}')
        self.get_logger().info(f'  DZ (Height)   : {dz_mm} mm')
        self.get_logger().info(f'  Plate Top Len : {plate_top_len} mm')
        self.get_logger().info(f'  Max Width     : {plate_max_width} mm')
        self.get_logger().info(f'  Operator ID   : {cr_op_indi_id}')
        self.get_logger().info(f'  Camera        : {"ON ✅" if camera_use else "OFF ❌"}')
        self.get_logger().info(f'  LiDAR         : {"ON ✅" if lidar_use else "OFF ❌"}')
        self.get_logger().info(f'  Timestamp     : {request.head.date} {request.head.time}')
        self.get_logger().info(f'  Raw Data     : {request.body.raw}')
        self.get_logger().info('=' * 60)
        
        # 비동기 호출
        future = self.acwl_client.call_async(request)
        
        return future
    
    def handle_response(self, future):
        """응답 처리"""
        try:
            response = future.result()
            
            self.get_logger().info('=' * 60)
            self.get_logger().info('📥 ACWL0001 Response Received:')
            
            if response.accepted:
                self.get_logger().info('  ✅ Status: ACCEPTED')
                self.get_logger().info('  📊 Task started. Check inference_node logs for progress.')
            else:
                self.get_logger().warning('  ❌ Status: REJECTED')
                self.get_logger().warning('  ⚠️  Inference node is busy or unavailable')
            
            self.get_logger().info('=' * 60 + '\n')
            
            return response.accepted
            
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
    print("\n" + "=" * 60)
    print("🎮 ACWL0001 Demo - Keyboard Control")
    print("=" * 60)
    print("📋 시나리오 선택:")
    print()
    print("  [1] 세로 측정 (Camera + LiDAR)")
    print("      - 2500mm x 1800mm 판재")
    print("      - 가장 정확한 3D 측정")
    print()
    print("  [2] 가로 측정 (Camera + LiDAR)")
    print("      - 3000mm x 2000mm 판재")
    print("      - 3D 측정 (가로 방향)")
    print()
    print("  [3] Camera Only")
    print("      - 2000mm x 1500mm 판재")
    print("      - 2D 픽셀 기반 측정")
    print()
    print("  [4] LiDAR Only")
    print("      - 2200mm x 1600mm 판재")
    print("      - 3D 포인트클라우드만 사용")
    print()
    print("  [5] 소형 판재 (크기 초과 테스트)")
    print("      - Max: 1000mm x 800mm")
    print("      - 실제 측정값이 더 크면 '0010' 에러")
    print()
    print("  [6] 대형 판재")
    print("      - Max: 5000mm x 3000mm")
    print("      - 여유 있는 크기 설정")
    print()
    print("  [7] 동시 요청 테스트 (거부 예상)")
    print("      - 이전 작업 진행 중 새 요청")
    print()
    print("  [8] 고속 연속 요청 (3회)")
    print("      - 빠른 연속 측정 시뮬레이션")
    print()
    print("  [9] 커스텀 입력")
    print("      - 사용자 정의 파라미터")
    print()
    print("  [h] 도움말 (메뉴 재출력)")
    print("  [q] 종료")
    print()
    print("=" * 60)
    print("키를 누르세요: ", end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    
    demo_client = ACWL0001DemoClient()
    
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
            # 시나리오 1: 세로 측정 (Camera + LiDAR)
            # ========================================
            if key == '1':
                demo_client.get_logger().info('\n🎬 [1] 세로 측정 (Camera + LiDAR)')
                future = demo_client.send_request(
                    eqp_cd='TBCRB4',
                    dx_direct=0,                # 정방향
                    dy_direct=0,                # 정방향
                    dz_mm=0,
                    plate_top_len=2500,
                    plate_max_width=1800,
                    cr_op_indi_id='OP001',
                    camera_use=True,
                    lidar_use=True
                )
            
            # ========================================
            # 시나리오 2: 가로 측정 (Camera + LiDAR)
            # ========================================
            elif key == '2':
                demo_client.get_logger().info('\n🎬 [2] 가로 측정 (Camera + LiDAR)')
                future = demo_client.send_request(
                    eqp_cd='CRANE_H01',
                    dx_direct=1,                # 역방향
                    dy_direct=0,                # 정방향
                    dz_mm=0,
                    plate_top_len=3000,
                    plate_max_width=2000,
                    cr_op_indi_id='OP002',
                    camera_use=True,
                    lidar_use=True
                )
            
            # ========================================
            # 시나리오 3: Camera만
            # ========================================
            elif key == '3':
                demo_client.get_logger().info('\n🎬 [3] Camera Only')
                future = demo_client.send_request(
                    eqp_cd='CRANE_CAM',
                    dx_direct=0,
                    dy_direct=0,
                    dz_mm=0,
                    plate_top_len=2000,
                    plate_max_width=1500,
                    cr_op_indi_id='OP003',
                    camera_use=True,
                    lidar_use=False
                )
            
            # ========================================
            # 시나리오 4: LiDAR만
            # ========================================
            elif key == '4':
                demo_client.get_logger().info('\n🎬 [4] LiDAR Only')
                future = demo_client.send_request(
                    eqp_cd='CRANE_LDR',
                    dx_direct=0,
                    dy_direct=0,
                    dz_mm=0,
                    plate_top_len=2200,
                    plate_max_width=1600,
                    cr_op_indi_id='OP004',
                    camera_use=False,
                    lidar_use=True
                )
            
            # ========================================
            # 시나리오 5: 소형 판재 (크기 초과 테스트)
            # ========================================
            elif key == '5':
                demo_client.get_logger().info('\n🎬 [5] 소형 판재 (크기 초과 테스트)')
                future = demo_client.send_request(
                    eqp_cd='CRANE_SMALL',
                    dx_direct=0,
                    dy_direct=0,
                    dz_mm=0,
                    plate_top_len=1000,   # 작은 최대값
                    plate_max_width=800,
                    cr_op_indi_id='OP005',
                    camera_use=True,
                    lidar_use=True
                )
            
            # ========================================
            # 시나리오 6: 대형 판재
            # ========================================
            elif key == '6':
                demo_client.get_logger().info('\n🎬 [6] 대형 판재')
                future = demo_client.send_request(
                    eqp_cd='CRANE_LARGE',
                    dx_direct=1,
                    dy_direct=0,
                    dz_mm=0,
                    plate_top_len=5000,   # 큰 최대값
                    plate_max_width=3000,
                    cr_op_indi_id='OP006',
                    camera_use=True,
                    lidar_use=True
                )
            
            # ========================================
            # 시나리오 7: 동시 요청 테스트
            # ========================================
            elif key == '7':
                demo_client.get_logger().info('\n🎬 [7] 동시 요청 테스트')
                
                # 첫 번째 요청
                future1 = demo_client.send_request(
                    eqp_cd='CRANE_FIRST',
                    dx_direct=0,
                    dy_direct=0,
                    dz_mm=0,
                    plate_top_len=2000,
                    plate_max_width=1500,
                    cr_op_indi_id='OP007',
                    camera_use=True,
                    lidar_use=True
                )
                
                rclpy.spin_until_future_complete(demo_client, future1, timeout_sec=2.0)
                if future1.done():
                    demo_client.handle_response(future1)
                
                # 0.5초 후 두 번째 요청 (거부될 가능성 높음)
                time.sleep(0.5)
                demo_client.get_logger().info('⏱️  0.5초 후 두 번째 요청...')
                
                future = demo_client.send_request(
                    eqp_cd='CRANE_SECOND',
                    dx_direct=0,
                    dy_direct=0,
                    dz_mm=0,
                    plate_top_len=2000,
                    plate_max_width=1500,
                    cr_op_indi_id='OP008',
                    camera_use=True,
                    lidar_use=True
                )
            
            # ========================================
            # 시나리오 8: 고속 연속 요청
            # ========================================
            elif key == '8':
                demo_client.get_logger().info('\n🎬 [8] 고속 연속 요청 (3회)')
                
                for i in range(3):
                    demo_client.get_logger().info(f'\n🔄 요청 {i+1}/3')
                    
                    future_batch = demo_client.send_request(
                        eqp_cd=f'CRANE_BATCH{i+1:02d}',
                        dx_direct=i % 2,  # 정방향/역방향 번갈아
                        dy_direct=0,
                        dz_mm=0,
                        plate_top_len=2000 + i * 100,
                        plate_max_width=1500 + i * 50,
                        cr_op_indi_id=f'OP{9+i:03d}',
                        camera_use=True,
                        lidar_use=True
                    )
                    
                    rclpy.spin_until_future_complete(demo_client, future_batch, timeout_sec=2.0)
                    if future_batch.done():
                        demo_client.handle_response(future_batch)
                    
                    if i < 2:
                        time.sleep(1.0)  # 1초 대기
                
                future = None  # 이미 처리됨
            
            # ========================================
            # 시나리오 9: 커스텀 입력
            # ========================================
            elif key == '9':
                demo_client.get_logger().info('\n🎬 [9] 커스텀 입력')
                print("\n설비 코드 (예: CRANE_CUSTOM): ", end='', flush=True)
                
                # 원래 터미널 설정 복원
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                
                eqp_cd = input().strip() or 'CRANE_CUSTOM'
                
                print("DX 방향 (0=정방향, 1=역방향, 기본=0): ", end='', flush=True)
                dx_direct = int(input().strip() or '0')
                
                print("DY 방향 (0=정방향, 1=역방향, 기본=0): ", end='', flush=True)
                dy_direct = int(input().strip() or '0')
                
                print("DZ 높이 (mm, 기본=0): ", end='', flush=True)
                dz_mm = int(input().strip() or '0')
                
                print("판재 상단 길이 (mm, 기본=2000): ", end='', flush=True)
                plate_top_len = int(input().strip() or '2000')
                
                print("판재 최대 폭 (mm, 기본=1500): ", end='', flush=True)
                plate_max_width = int(input().strip() or '1500')
                
                print("작업 지시 ID (예: OP999): ", end='', flush=True)
                cr_op_indi_id = input().strip() or 'OP999'
                
                print("Camera 사용 (y/n, 기본=y): ", end='', flush=True)
                camera_use = input().strip().lower() != 'n'
                
                print("LiDAR 사용 (y/n, 기본=y): ", end='', flush=True)
                lidar_use = input().strip().lower() != 'n'
                
                future = demo_client.send_request(
                    eqp_cd=eqp_cd,
                    dx_direct=dx_direct,
                    dy_direct=dy_direct,
                    dz_mm=dz_mm,
                    plate_top_len=plate_top_len,
                    plate_max_width=plate_max_width,
                    cr_op_indi_id=cr_op_indi_id,
                    camera_use=camera_use,
                    lidar_use=lidar_use
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
    
    finally:
        # 터미널 설정 복원
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        
        demo_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
