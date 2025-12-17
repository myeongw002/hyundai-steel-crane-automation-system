#!/usr/bin/env python3
# socket_node.py
import rclpy
from rclpy.node import Node
import socket
import threading
from hyundai_steel_crane_automation_system.srv import ACWL0001, WLAC0001
from std_msgs.msg import Bool  # ✅ 추가
from protocol_parser import ProtocolParser
from datetime import datetime
import time

MIN_HEADER_FOR_LEN = 31  # MSG_LEN 끝 인덱스

class SocketBridgeNode(Node):
    def __init__(self):
        super().__init__('socket_bridge_node')

        self.declare_parameter('listen_ip', '0.0.0.0')
        self.declare_parameter('listen_port', 5000)
        self.declare_parameter('plc_ip', '192.168.1.100')  # ✅ PLC IP 파라미터
        self.declare_parameter('buffer_size', 4096)

        self.parser = ProtocolParser()

        # ROS client (ACWL0001 -> worker service)
        self.acwl_client = self.create_client(ACWL0001, 'acwl0001_service')
        if not self.acwl_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('acwl0001_service not available at startup')

        # ROS service (WLAC0001 from other nodes -> we send to PLC)
        self.wlac_service = self.create_service(WLAC0001, 'wlac0001_service', self.wlac_callback)

        # ✅ 추가: Heartbeat 퍼블리셔 (5Hz)
        self.heartbeat_pub = self.create_publisher(Bool, '/heartbeat/socket', 10)
        self.heartbeat_timer = self.create_timer(0.2, self._publish_heartbeat)  # 5Hz = 0.2초
        self.heartbeat_state = True  # 토글용
        self.get_logger().info('Heartbeat publisher initialized (5Hz)')

        # ✅ 수정: 연결된 클라이언트 관리 (소켓과 주소 저장)
        self.connected_clients = {}  # {socket: (ip, port)} 딕셔너리
        self.clients_lock = threading.Lock()

        # TCP server
        self.server_socket = None
        self.accept_thread = None
        self.client_threads = []
        self._start_socket_server()

    # ✅ 추가: Heartbeat 발행 함수
    def _publish_heartbeat(self):
        """5Hz로 heartbeat 발행"""
        msg = Bool()
        msg.data = self.heartbeat_state
        self.heartbeat_pub.publish(msg)
        self.heartbeat_state = not self.heartbeat_state  # True/False 토글

    # ----------------- Client management helpers -----------------
    def send_to_clients(self, data: bytes):
        """PLC IP에 해당하는 클라이언트에게만 데이터 전송"""
        plc_ip = self.get_parameter('plc_ip').value
        
        with self.clients_lock:
            if not self.connected_clients:
                self.get_logger().warn('No connected clients to send data')
                raise Exception('No connected clients')
            
            failed_clients = []
            success_count = 0
            target_found = False
            
            for client_sock, (ip, port) in self.connected_clients.items():
                # PLC IP와 일치하는 클라이언트에게만 전송
                if ip == plc_ip:
                    target_found = True
                    try:
                        client_sock.sendall(data)
                        success_count += 1
                        self.get_logger().info(f'WLAC packet data: {data.decode("ascii", errors="ignore")}')
                        self.get_logger().info(f'Sent {len(data)} bytes to PLC client {ip}:{port}')
                    except Exception as e:
                        self.get_logger().error(f'Failed to send to PLC client {ip}:{port}: {e}')
                        failed_clients.append(client_sock)
            
            # 실패한 클라이언트 제거
            for client_sock in failed_clients:
                addr = self.connected_clients.pop(client_sock, None)
                try:
                    client_sock.close()
                except: pass
                if addr:
                    self.get_logger().warn(f'Removed failed client {addr[0]}:{addr[1]}')
            
            if not target_found:
                raise Exception(f'PLC client with IP {plc_ip} not connected')
            
            if success_count == 0:
                raise Exception(f'Failed to send to PLC client {plc_ip}')

    # ----------------- TCP server -----------------
    def _start_socket_server(self):
        ip = self.get_parameter('listen_ip').value
        port = int(self.get_parameter('listen_port').value)
        bufsize = int(self.get_parameter('buffer_size').value)

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((ip, port))
        s.listen(8)
        s.settimeout(1.0)
        self.server_socket = s
        self.get_logger().info(f'Socket server listening on {ip}:{port}')

        self.accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self.accept_thread.start()

    def _accept_loop(self):
        while rclpy.ok():
            try:
                client_sock, addr = self.server_socket.accept()
                client_sock.settimeout(2.0)
                client_ip, client_port = addr
                self.get_logger().info(f'Accepted connection from {client_ip}:{client_port}')
                
                # ✅ 클라이언트 딕셔너리에 추가 (소켓: (ip, port))
                with self.clients_lock:
                    self.connected_clients[client_sock] = (client_ip, client_port)
                
                t = threading.Thread(target=self._client_loop, args=(client_sock, addr), daemon=True)
                t.start()
                self.client_threads.append(t)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'Accept loop error: {e}')
                time.sleep(0.5)

    def _client_loop(self, client_sock: socket.socket, addr):
        buf = bytearray()
        bs = int(self.get_parameter('buffer_size').value)
        try:
            while rclpy.ok():
                try:
                    chunk = client_sock.recv(bs)
                    if not chunk:
                        self.get_logger().info(f'Connection closed by {addr}')
                        break
                    buf.extend(chunk)
                except socket.timeout:
                    continue

                # 프레이밍: MSG_LEN 위치에서 전체 길이 읽어 packet 단위로 분리
                while len(buf) >= MIN_HEADER_FOR_LEN:
                    try:
                        raw_len_bytes = bytes(buf[27:31])
                        msg_len = int(raw_len_bytes.decode('ascii', errors='ignore'))
                    except ValueError:
                        # 동기화 복구: 검색 가능한 시그니처로 이동
                        pos_acwl = buf.find(b'ACWL')
                        pos_wlac = buf.find(b'WLAC')
                        pos = -1
                        if pos_acwl != -1: pos = pos_acwl
                        if pos_wlac != -1 and (pos == -1 or pos_wlac < pos): pos = pos_wlac
                        if pos > 0:
                            del buf[:pos]
                            continue
                        break

                    if msg_len <= 0:
                        # invalid -> drop initial byte and resync
                        del buf[:1]
                        continue

                    if len(buf) < msg_len:
                        break  # wait for more

                    packet = bytes(buf[:msg_len])
                    del buf[:msg_len]
                    # dispatch
                    try:
                        self._dispatch_packet(packet, client_sock)
                    except Exception as e:
                        self.get_logger().error(f'Packet dispatch error: {e}')
                        continue

        finally:
            # ✅ 클라이언트 딕셔너리에서 제거
            with self.clients_lock:
                if client_sock in self.connected_clients:
                    removed_addr = self.connected_clients.pop(client_sock)
                    self.get_logger().info(f'Client {removed_addr[0]}:{removed_addr[1]} disconnected and removed')
            
            try:
                client_sock.close()
            except: pass

    def _dispatch_packet(self, packet: bytes, client_sock: socket.socket):
        # decode per-packet
        data = packet.decode('ascii', errors='ignore')
        self.get_logger().info(f'Received packet data: {data}')
        # 메시지 타입 확인 (처음 4글자)
        msg_type = data[:4] if len(data) >= 4 else ""
        
        if msg_type == "WLAC":
            # WLAC 메시지면 ARK 확인하고 처리 안함
            if "ARK" in data:
                self.get_logger().info('WLAC ARK received - No processing needed')
            else:
                self.get_logger().info('WLAC received but no ARK - Ignoring')
            return  # 다음 메시지 대기
        
        elif msg_type == "ACWL":
            # ACWL 메시지면 ARK 붙여서 응답
            self.get_logger().info('ACWL received - Sending ARK response')
            
            try:
                # 응답 전송: 처음 40 byte + 공백 17 byte + 'ARK' 3 byte = 총 60 byte
                if len(packet) >= 40:
                    header = bytearray(packet[:40])  # bytearray로 변환
                else:
                    header = bytearray(packet + b' ' * (40 - len(packet)))
                
                # msg_len을 '0060'으로 수정 (27~30 인덱스, 4글자)
                header[27:31] = b'0060'
                
                # 60바이트 맞추기: header(40) + ARK(3) + 공백(17) = 60
                response = bytes(header) + b'ARK' + b' ' * 17
                
                client_sock.send(response)
                self.get_logger().info(f'Sent ARK response: {len(response)} bytes')
            except Exception as e:
                self.get_logger().error(f'Failed to send ARK response: {e}')
            
            # ACWL 처리 (worker service 호출)
            self._handle_acwl_packet(packet)
        

    # ----------------- ACWL handling (call worker service) -----------------
    def _handle_acwl_packet(self, packet: bytes):
        try:
            req = self.parser.parse_acwl0001(packet)
            # check service readiness
            if not self.acwl_client.service_is_ready():
                self.get_logger().error('ACWL service not ready')
                return
            fut = self.acwl_client.call_async(req)
            fut.add_done_callback(self._acwl_response_callback)
        except Exception as e:
            self.get_logger().error(f'parse_acwl0001 error: {e}')

    def _acwl_response_callback(self, fut):
        try:
            resp = fut.result()
            if resp.accepted:
                self.get_logger().info('ACWL accepted by processor')
            else:
                self.get_logger().warn('ACWL rejected by processor: ' + (resp.reason or ''))
        except Exception as e:
            self.get_logger().error(f'ACWL service call failed: {e}')

    # ----------------- WLAC service callback (ROS -> Clients) -----------------
    def wlac_callback(self, request, response):
        try:
            # ensure header date/time
            if not request.head.date or not request.head.time:
                now = datetime.utcnow()
                request.head.date = now.strftime('%Y%m%d')
                request.head.time = now.strftime('%H%M%S')

            payload = self.parser.build_wlac0001(request)
            # ✅ 수정: TCP 서버를 통해 연결된 클라이언트들에게 전송
            self.send_to_clients(payload)
            response.stored = True
            response.error = ''
        except Exception as e:
            self.get_logger().error(f'WLAC callback error: {e}')
            response.stored = False
            response.error = str(e)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SocketBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.get_logger().info('Shutting down')
        try:
            if node.server_socket:
                node.server_socket.close()
        except: pass
        # ✅ 추가: 타이머 정리
        try:
            node.heartbeat_timer.cancel()
        except: pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
