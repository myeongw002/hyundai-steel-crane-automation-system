"""
ftp_uploader.py - SFTP upload utilities for measurement results
Provides functionality to upload result files to remote SFTP server
"""

import os
import paramiko
import time
import posixpath
from typing import List, Optional
import logging


class FTPUploader:
    """
    SFTP 업로더 클래스
    측정 결과 파일을 원격 SFTP 서버로 업로드
    """
    
    def __init__(self, host: str, username: str, password: str, remote_base_dir: str = 'upload', port: int = 22):
        """
        Args:
            host: SFTP 서버 주소
            username: SFTP 계정
            password: SFTP 비밀번호
            remote_base_dir: 원격 기본 디렉토리
            port: SFTP 포트 (기본값: 22)
        """
        self.host = host
        self.username = username
        self.password = password
        self.remote_base_dir = remote_base_dir
        self.port = port
        self.logger = logging.getLogger(__name__)

    def _mkdir_p(self, sftp: paramiko.SFTPClient, remote_dir: str):
        """remote_dir을 / 기준으로 부모부터 재귀 생성 (상대/절대 모두 처리)"""
        remote_dir = remote_dir.strip('/')
        if not remote_dir:
            return
        parts = remote_dir.split('/')
        cur = ''
        for p in parts:
            cur = p if cur == '' else f"{cur}/{p}"
            try:
                sftp.stat(cur)
            except FileNotFoundError:
                sftp.mkdir(cur)

    def _verify_size_with_retry(self, sftp: paramiko.SFTPClient, remote_path: str,
                                expected_size: int, retries: int = 10, sleep_sec: float = 0.2) -> bool:
        """일부 서버의 stat 반영 지연 대비: 짧게 여러 번 확인"""
        for _ in range(retries):
            try:
                sz = sftp.stat(remote_path).st_size
                if sz == expected_size:
                    return True
            except FileNotFoundError:
                pass
            time.sleep(sleep_sec)
        return False

    def _put_file_strict(self, sftp: paramiko.SFTPClient, local_path: str, remote_path: str):
        """업로드 + (필요시) 지연 고려한 크기 검증"""
        local_size = os.path.getsize(local_path)

        # 1) 전송 (confirm=False로 Paramiko 내부 즉시-stat 검증을 피함)
        sftp.put(local_path, remote_path, confirm=False)

        # 2) 크기 검증(재시도)
        ok = self._verify_size_with_retry(sftp, remote_path, local_size)
        if not ok:
            # 실패 시 0바이트 찌꺼기 제거 시도
            try:
                sftp.remove(remote_path)
            except Exception:
                pass
            raise IOError(f"size mismatch after upload: expected={local_size}, remote!=expected")
    
    def upload_results(self, local_base_dir: str, sequence_id: str) -> dict:
        result = {'success': False, 'uploaded_files': [], 'errors': []}
        local_results_dir = os.path.join(local_base_dir, sequence_id, 'results')
        if not os.path.isdir(local_results_dir):
            result['errors'].append(f'Local results directory not found: {local_results_dir}')
            return result

        ssh = None
        sftp = None
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(self.host, port=self.port, username=self.username,
                        password=self.password, timeout=10)
            sftp = ssh.open_sftp()

            # base dir로 이동/생성
            try:
                sftp.chdir(self.remote_base_dir)
            except IOError:
                self._mkdir_p(sftp, self.remote_base_dir)
                sftp.chdir(self.remote_base_dir)

            # frame_0001_result.png 파일을 스케줄ID.png로 업로드
            target_file = 'frame_0001_result.png'
            lp = os.path.join(local_results_dir, target_file)
            
            if os.path.isfile(lp):
                # 원격 파일명을 스케줄ID.png로 설정
                remote_filename = f'{sequence_id}.png'
                try:
                    self._put_file_strict(sftp, lp, remote_filename)
                    result['uploaded_files'].append(remote_filename)
                    result['success'] = True
                except Exception as e:
                    result['errors'].append(f'Failed to upload {lp}: {e}')
            else:
                result['errors'].append(f'{target_file} not found in {local_results_dir}')

        except Exception as e:
            result['errors'].append(f'SFTP unexpected error: {e}')
        finally:
            try:
                if sftp: sftp.close()
            finally:
                if ssh: ssh.close()

        return result
    
    def _ensure_remote_directory(self, sftp: paramiko.SFTPClient, remote_dir: str):
        """
        원격 디렉토리가 존재하는지 확인하고 없으면 생성
        
        Args:
            sftp: SFTP 클라이언트 객체
            remote_dir: 원격 디렉토리 경로
        """
        try:
            sftp.stat(remote_dir)
        except IOError:
            # 디렉토리가 없으면 생성
            try:
                sftp.mkdir(remote_dir)
                self.logger.info(f'📁 Created remote directory: {remote_dir}')
            except IOError as e:
                # 이미 존재하거나 권한 문제
                self.logger.warning(f'Could not create directory {remote_dir}: {e}')
    
    def _upload_directory_recursive(self, sftp: paramiko.SFTPClient, local_dir: str, 
                                   remote_dir: str, result: dict) -> int:
        """
        디렉토리를 재귀적으로 업로드 (파일 구조 유지)
        
        Args:
            sftp: SFTP 클라이언트 객체
            local_dir: 로컬 디렉토리 경로
            remote_dir: 원격 디렉토리 경로
            result: 결과 딕셔너리 (업데이트됨)
        
        Returns:
            업로드된 파일 개수
        """
        uploaded_count = 0
        
        for item in os.listdir(local_dir):
            local_path = os.path.join(local_dir, item)
            remote_path = f'{remote_dir}/{item}'
            
            if os.path.isfile(local_path):
                # 파일 업로드
                try:
                    sftp.put(local_path, remote_path)
                    result['uploaded_files'].append(remote_path)
                    uploaded_count += 1
                    self.logger.info(f'📤 Uploaded: {remote_path}')
                except Exception as e:
                    error_msg = f'Failed to upload {local_path}: {str(e)}'
                    self.logger.error(error_msg)
                    result['errors'].append(error_msg)
            
            elif os.path.isdir(local_path):
                # 서브 디렉토리 생성 및 재귀 업로드
                self._ensure_remote_directory(sftp, remote_path)
                count = self._upload_directory_recursive(sftp, local_path, remote_path, result)
                uploaded_count += count
        
        return uploaded_count
    
    def test_connection(self) -> bool:
        """
        SFTP 연결 테스트
        
        Returns:
            연결 성공 여부
        """
        ssh_client = None
        try:
            ssh_client = paramiko.SSHClient()
            ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh_client.connect(
                hostname=self.host,
                port=self.port,
                username=self.username,
                password=self.password,
                timeout=10
            )
            sftp = ssh_client.open_sftp()
            sftp.close()
            ssh_client.close()
            self.logger.info(f'✅ SFTP connection test successful: {self.host}:{self.port}')
            return True
        except Exception as e:
            self.logger.error(f'❌ SFTP connection test failed: {e}')
            return False
        finally:
            if ssh_client:
                try:
                    ssh_client.close()
                except:
                    pass


def create_ftp_uploader(host: str = '172.29.73.49', 
                       username: str = 'plateftp',
                       password: str = 'plateftp',
                       remote_base_dir: str = 'upload',
                       port: int = 22) -> FTPUploader:
    """
    기본 설정으로 FTPUploader 인스턴스 생성 (SFTP 사용)
    
    Args:
        host: SFTP 서버 주소
        username: SFTP 계정
        password: SFTP 비밀번호
        remote_base_dir: 원격 기본 디렉토리
        port: SFTP 포트 (기본값: 22)
    
    Returns:
        FTPUploader 인스턴스
    """
    return FTPUploader(host, username, password, remote_base_dir, port)
