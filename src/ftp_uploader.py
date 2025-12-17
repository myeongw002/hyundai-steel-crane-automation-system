"""
ftp_uploader.py - SFTP upload utilities for measurement results
Provides functionality to upload result files to remote SFTP server
"""

import os
import paramiko
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
    
    def upload_results(self, local_base_dir: str, sequence_id: str) -> dict:
        """
        결과 파일들을 SFTP 서버로 업로드
        
        Args:
            local_base_dir: 로컬 기본 디렉토리 경로
            sequence_id: 시퀀스 ID (스케줄ID)
        
        Returns:
            업로드 결과 딕셔너리 {'success': bool, 'uploaded_files': list, 'errors': list}
        """
        result = {
            'success': False,
            'uploaded_files': [],
            'errors': []
        }
        
        # 로컬 results 디렉토리 경로
        local_results_dir = os.path.join(local_base_dir, sequence_id, 'results')
        
        if not os.path.exists(local_results_dir):
            error_msg = f'Local results directory not found: {local_results_dir}'
            self.logger.error(error_msg)
            result['errors'].append(error_msg)
            return result
        
        # SFTP 연결
        ssh_client = None
        sftp = None
        try:
            # SSH 클라이언트 생성
            ssh_client = paramiko.SSHClient()
            ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            
            # 연결
            ssh_client.connect(
                hostname=self.host,
                port=self.port,
                username=self.username,
                password=self.password,
                timeout=10
            )
            sftp = ssh_client.open_sftp()
            self.logger.info(f'✅ Connected to SFTP server: {self.host}:{self.port}')
            
            # 원격 기본 디렉토리로 이동
            try:
                sftp.chdir(self.remote_base_dir)
            except IOError:
                # 디렉토리가 없으면 생성
                sftp.mkdir(self.remote_base_dir)
                sftp.chdir(self.remote_base_dir)
            
            # 원격 시퀀스ID/results 디렉토리 생성
            remote_seq_dir = sequence_id
            remote_results_dir = f'{sequence_id}/results'
            
            self._ensure_remote_directory(sftp, remote_seq_dir)
            self._ensure_remote_directory(sftp, remote_results_dir)
            
            # results 디렉토리 아래 모든 파일 업로드
            uploaded_count = self._upload_directory_recursive(
                sftp, local_results_dir, remote_results_dir, result
            )
            
            if uploaded_count > 0:
                result['success'] = True
                self.logger.info(f'✅ Successfully uploaded {uploaded_count} files')
            else:
                result['errors'].append('No files were uploaded')
            
        except paramiko.AuthenticationException as e:
            error_msg = f'SFTP authentication failed: {str(e)}'
            self.logger.error(error_msg)
            result['errors'].append(error_msg)
        except paramiko.SSHException as e:
            error_msg = f'SFTP SSH error: {str(e)}'
            self.logger.error(error_msg)
            result['errors'].append(error_msg)
        except Exception as e:
            error_msg = f'SFTP unexpected error: {str(e)}'
            self.logger.error(error_msg)
            result['errors'].append(error_msg)
        finally:
            if sftp:
                try:
                    sftp.close()
                except:
                    pass
            if ssh_client:
                try:
                    ssh_client.close()
                except:
                    pass
        
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
