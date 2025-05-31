import subprocess
import time
import os
import carla
import socket
import signal as process_signal
from ..utils.logging_utils import setup_logger

CARLA_PATH = "/home/w/workspace/carla_apollo/CARLA_0.9.14/CarlaUE4.sh"
WINDOWED = True



class CarlaServerManager:
    def __init__(self):

        self.logger = setup_logger('CarlaServer')
        self.carla_path = CARLA_PATH
        self.windowed = WINDOWED
        self.carla_process = None


    def start_carla(self):
        """Start CARLA server"""
        try:
            print("Starting CARLA server...")
            cmd = [self.carla_path] #, "-quality-level=Low"]
            
            # Add parameters if windowed mode is specified
            if self.windowed:
                # cmd.extend(["-windowed", "-ResX=800", "-ResY=600"])
                cmd.extend(["-RenderOffScreen"])
            self.carla_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            time.sleep(5)  # Wait for CARLA to start
            
            # Test connection
            client = carla.Client('localhost', 2000)
            client.set_timeout(10.0)
            world = client.get_world()
            print(f"Successfully connected to CARLA, current map: {world.get_map().name}")
            
        except Exception as e:
            print(f"Failed to start CARLA: {e}")
            if self.carla_process:
                self.stop_carla()
            raise
    

    def stop_carla(self):
        """Stop CARLA server and related processes"""
        try:
            # Use process_signal for process management
            ports_to_check = [2000, 8000]
            for port in ports_to_check:
                try:
                    # Use lsof on Linux/Mac
                    if os.name != 'nt':  # Non-Windows systems
                        cmd = f"lsof -ti:{port}"
                        pids = subprocess.check_output(cmd, shell=True).decode().strip().split('\n')
                        for pid in pids:
                            if pid:  # Ensure pid is not empty
                                os.kill(int(pid), process_signal.SIGTERM)
                                self.logger.info(f"Terminated process on port {port} (PID: {pid})")
                    else:  # Windows systems
                        cmd = f"netstat -ano | findstr :{port}"
                        output = subprocess.check_output(cmd, shell=True).decode()
                        for line in output.split('\n'):
                            if line.strip():
                                pid = line.strip().split()[-1]
                                subprocess.run(f"taskkill /F /PID {pid}", shell=True)
                                self.logger.info(f"Terminated process on port {port} (PID: {pid})")
                except subprocess.CalledProcessError:
                    self.logger.debug(f"No running processes on port {port}")
                except Exception as e:
                    self.logger.error(f"Error closing process on port {port}: {e}")

            # Close CARLA server process
            if hasattr(self, 'carla_process') and self.carla_process:
                self.carla_process.terminate()
                try:
                    self.carla_process.wait(timeout=5)  # Wait for process termination
                except subprocess.TimeoutExpired:
                    self.carla_process.kill()  # Force kill if process doesn't terminate in time
                self.carla_process = None
                self.logger.info("CARLA server stopped")

            # Additional check to ensure all related processes are closed
            time.sleep(1)  # Wait for processes to fully close
            self._verify_ports_closed(ports_to_check)
        except Exception as e:
            self.logger.error(f"Error stopping CARLA: {e}")
            self.logger.exception("Detailed error information:")
            
    def _verify_ports_closed(self, ports):
        """Verify if specified ports are closed"""
        for port in ports:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                result = sock.connect_ex(('127.0.0.1', port))
                sock.close()
                if result == 0:
                    self.logger.warning(f"Port {port} may still be occupied")
                    # Can optionally try to close processes again here
                else:
                    self.logger.debug(f"Port {port} successfully closed")
            except Exception as e:
                self.logger.error(f"Error checking port {port} status: {e}")
         