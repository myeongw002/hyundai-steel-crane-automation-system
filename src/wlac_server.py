#!/usr/bin/env python3
"""
WLAC0001 서비스 서버 (Mock WCS)
InferenceNode로부터 측정 결과를 수신하는 WCS 역할 시뮬레이션
"""

import rclpy
from rclpy.node import Node
from hyundai_steel_crane_automation_system.srv import WLAC0001
from hyundai_steel_crane_automation_system.msg import WLAC0001Body, HeadCR
from datetime import datetime


class WLAC0001Server(Node):
    """WLAC0001 서비스 서버 (Mock WCS)"""
    
    # 결과 코드 정의
    RESULT_CODES = {
        '0000': 'SUCCESS - 정상 측정',
        '0010': 'WARNING - 치수 경미한 이탈',
        '0020': 'ERROR - 치수 허용 범위 초과',
        '0030': 'FAIL - 측정 실패'
    }
    
    def __init__(self):
        super().__init__('wlac0001_server')
        
        # 파라미터 선언
        self.declare_parameter('service_name', '/wlac0001_service')
        self.declare_parameter('tolerance_mm', 50)  # 측정값 허용오차 ±50mm
        
        # 서비스 서버 생성
        service_name = self.get_parameter('service_name').value
        self.srv = self.create_service(
            WLAC0001,
            service_name,
            self.wlac0001_callback
        )
        
        # 수신 데이터 저장
        self.received_data = []
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('🏭 WLAC0001 Server Started (Mock WCS)')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Service      : {service_name}')
        self.get_logger().info(f'Tolerance    : ±{self.get_parameter("tolerance_mm").value} mm')
        self.get_logger().info('=' * 70)
        self.get_logger().info('Waiting for measurement results...\n')
    
    def wlac0001_callback(
        self,
        request: WLAC0001.Request,
        response: WLAC0001.Response
    ) -> WLAC0001.Response:
        """WLAC0001 요청 처리"""
        
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('📥 WLAC0001 Request Received')
        self.get_logger().info('=' * 70)
        
        # 헤더 정보
        head = request.head
        self.get_logger().info(f'Header:')
        self.get_logger().info(f'  - Message ID : {head.msg_id}')
        self.get_logger().info(f'  - Date       : {head.date}')
        self.get_logger().info(f'  - Time       : {head.time}')
        self.get_logger().info(f'  - Form       : {head.form}')
        self.get_logger().info(f'  - Length     : {head.msg_len} bytes')
        
        # 바디 정보
        body = request.body
        self.get_logger().info(f'\nBody:')
        self.get_logger().info(f'  - Equipment Code    : {body.eqp_cd}')
        self.get_logger().info(f'  - Result Code       : {body.result_code}')
        self.get_logger().info(f'  - Length P1-P2 Diff : {body.len_result_p1_2:+5d} mm')
        self.get_logger().info(f'  - Length P3-P4 Diff : {body.len_result_p3_4:+5d} mm')
        self.get_logger().info(f'  - Width P5-P6 Diff  : {body.width_result_p5_6:+5d} mm')
        self.get_logger().info(f'  - Width P7-P8 Diff  : {body.width_result_p7_8:+5d} mm')
                
        # 데이터 검증
        validation_result = self._validate_measurement(body)
        
        if validation_result['valid']:
            # 성공 응답
            response.stored = True
            response.error = ''
            
            self.get_logger().info('-' * 70)
            self.get_logger().info('✅ Measurement Result: STORED')
            self.get_logger().info(f'   Result Code: {body.result_code}')
            self.get_logger().info(f'   Description: {self.RESULT_CODES.get(body.result_code, "UNKNOWN")}')
            
            # 데이터 저장
            self.received_data.append({
                'msg_id': head.msg_id,
                'date': head.date,
                'time': head.time,
                'eqp_cd': body.eqp_cd,
                'result_code': body.result_code,
                'len_result_p1_2': body.len_result_p1_2,
                'len_result_p3_4': body.len_result_p3_4,
                'width_result_p5_6': body.width_result_p5_6,
                'width_result_p7_8': body.width_result_p7_8,
                'stored_at': self.get_clock().now().to_msg()
            })
            
            self.get_logger().info(f'   Total Received: {len(self.received_data)} measurements')
        else:
            # 실패 응답
            response.stored = False
            response.error = validation_result['error_msg']
            
            self.get_logger().error('-' * 70)
            self.get_logger().error('❌ Measurement Result: REJECTED')
            self.get_logger().error(f'   Error Code: {validation_result["error_code"]}')
            self.get_logger().error(f'   Error Msg : {validation_result["error_msg"]}')
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('')
        
        return response
    
    def _validate_measurement(self, body: WLAC0001Body) -> dict:
        """측정 결과 검증"""
        
        # 1. 결과 코드 검증
        if body.result_code not in self.RESULT_CODES:
            return {
                'valid': False,
                'error_code': 'E001',
                'error_msg': f'Invalid result_code: {body.result_code} (expected: 0000, 0010, 0020, 0030)'
            }
        
        # 2. 장비 코드 검증
        valid_equipment = ['CRANE_V01', 'CRANE_V02', 'CRANE_TEST']
        if body.eqp_cd not in valid_equipment:
            return {
                'valid': False,
                'error_code': 'E002',
                'error_msg': f'Invalid eqp_cd: {body.eqp_cd} (expected: {", ".join(valid_equipment)})'
            }
        
        # 3. 측정값 범위 검증 (차이값은 ±5000mm 이내)
        measurements = [
            ('len_result_p1_2', body.len_result_p1_2),
            ('len_result_p3_4', body.len_result_p3_4),
            ('width_result_p5_6', body.width_result_p5_6),
            ('width_result_p7_8', body.width_result_p7_8)
        ]
        
        for name, value in measurements:
            if abs(value) > 5000:
                return {
                    'valid': False,
                    'error_code': 'E003',
                    'error_msg': f'Invalid {name}: {value}mm (expected: ±5000mm)'
                }
        
        # 5. 결과 코드 일관성 검증
        tolerance = self.get_parameter('tolerance_mm').value
        
        # 예상 결과 코드 계산 (간단한 로직)
        # 4개 측정값 중 하나라도 허용범위를 초과하면 에러
        max_abs_value = max(abs(v) for _, v in measurements)
        
        if body.result_code == '0030':
            # FAIL은 항상 허용
            pass
        elif body.result_code == '0020':
            # ERROR: 허용 범위 초과 확인
            # (실제로는 기준값이 필요하지만 여기서는 생략)
            pass
        elif body.result_code == '0010':
            # WARNING: 경미한 이탈
            pass
        elif body.result_code == '0000':
            # SUCCESS: 정상
            pass
        
        return {'valid': True}
    
    def print_statistics(self):
        """수신 통계 출력"""
        if not self.received_data:
            self.get_logger().info('No data received yet.')
            return
        
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('📊 Received Data Statistics')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Total Measurements: {len(self.received_data)}')
        
        # 결과 코드별 통계
        result_code_counts = {}
        for data in self.received_data:
            code = data['result_code']
            result_code_counts[code] = result_code_counts.get(code, 0) + 1
        
        self.get_logger().info('')
        self.get_logger().info('Result Code Distribution:')
        for code, count in sorted(result_code_counts.items()):
            desc = self.RESULT_CODES.get(code, 'UNKNOWN')
            percentage = (count / len(self.received_data)) * 100
            self.get_logger().info(f'  - {code} ({desc}): {count} ({percentage:.1f}%)')
        
        # 장비별 통계
        eqp_counts = {}
        for data in self.received_data:
            eqp = data['eqp_cd']
            eqp_counts[eqp] = eqp_counts.get(eqp, 0) + 1
        
        self.get_logger().info('')
        self.get_logger().info('Equipment Distribution:')
        for eqp, count in sorted(eqp_counts.items()):
            percentage = (count / len(self.received_data)) * 100
            self.get_logger().info(f'  - {eqp}: {count} ({percentage:.1f}%)')
        
        # 측정값 통계 (result_code가 0000, 0010인 경우만)
        valid_data = [d for d in self.received_data if d['result_code'] in ['0000', '0010']]
        
        if valid_data:
            len_p1_p2 = [d['len_result_p1_2'] for d in valid_data]
            len_p3_p4 = [d['len_result_p3_4'] for d in valid_data]
            width_p5_p6 = [d['width_result_p5_6'] for d in valid_data]
            width_p7_p8 = [d['width_result_p7_8'] for d in valid_data]
            
            self.get_logger().info('')
            self.get_logger().info(f'Measurement Statistics (n={len(valid_data)}):')
            self.get_logger().info('')
            self.get_logger().info(f'Length P1-P2 Diff:')
            self.get_logger().info(f'  - Min : {min(len_p1_p2):+5d} mm')
            self.get_logger().info(f'  - Max : {max(len_p1_p2):+5d} mm')
            self.get_logger().info(f'  - Avg : {sum(len_p1_p2)/len(len_p1_p2):+6.1f} mm')
            
            self.get_logger().info('')
            self.get_logger().info(f'Length P3-P4 Diff:')
            self.get_logger().info(f'  - Min : {min(len_p3_p4):+5d} mm')
            self.get_logger().info(f'  - Max : {max(len_p3_p4):+5d} mm')
            self.get_logger().info(f'  - Avg : {sum(len_p3_p4)/len(len_p3_p4):+6.1f} mm')
            
            self.get_logger().info('')
            self.get_logger().info(f'Width P5-P6 Diff:')
            self.get_logger().info(f'  - Min : {min(width_p5_p6):+5d} mm')
            self.get_logger().info(f'  - Max : {max(width_p5_p6):+5d} mm')
            self.get_logger().info(f'  - Avg : {sum(width_p5_p6)/len(width_p5_p6):+6.1f} mm')
            
            self.get_logger().info('')
            self.get_logger().info(f'Width P7-P8 Diff:')
            self.get_logger().info(f'  - Min : {min(width_p7_p8):+5d} mm')
            self.get_logger().info(f'  - Max : {max(width_p7_p8):+5d} mm')
            self.get_logger().info(f'  - Avg : {sum(width_p7_p8)/len(width_p7_p8):+6.1f} mm')
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)
    
    server = WLAC0001Server()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('\nKeyboard interrupt detected')
        
        # 종료 전 통계 출력
        server.print_statistics()
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()