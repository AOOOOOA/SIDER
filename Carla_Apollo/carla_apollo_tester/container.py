import os
import subprocess
from pathlib import Path

import docker
import docker.errors

from config import DOCKER_CMD, PROJECT_NAME, PROJECT_ROOT, SCRIPTS,CARLA_SIMULATOR,GENERATE_TRAFFIC, CARLA_RECORD_DATA
from utils import run_command
import time 
from loguru import logger
from path_manager import PathManager
import threading


#TODO: seperate the functoion running within the docker and on the PC 

class ApolloContainer:
    def __init__(self, apollo_dir: Path, ctn_name: str) -> None:
        self.apollo_dir: Path = apollo_dir
        self.ctn_name: str = ctn_name
        logger.debug(f"Initialized ApolloContainer with name: {ctn_name}")
        self.bridge_path = Path(self.apollo_dir, "modules", "carla_bridge")
        self.processes=[]

    def __repr__(self):
        return f"ApolloContainer(ctn_name={self.ctn_name})"

    def start_container(self, start_script=SCRIPTS.MULTI_CTN_DEV_START, verbose=False):
    # def start_container(self, start_script=SCRIPTS.DEV_START, verbose=False):
        
        if self.is_running():
            return True
        options = "-y -l -f"
        # docker_script_dir = Path(self.apollo_dir, "docker", "scripts")
        docker_script_dir = Path(self.apollo_dir, "docker", "scripts")
        
        cmd = f"bash {start_script} {options}"
        try:
            result=subprocess.run(
                cmd,
                env={
                    "CURR_DIR": docker_script_dir,
                    "DEV_CONTAINER": self.ctn_name,
                    "USER": os.environ.get("USER"),
                    # "PROJECT_ROOT": PROJECT_ROOT.absolute(),
                    "PROJECT_ROOT": self.apollo_dir, #FIXME: 这里改了, 可能会改出问题
                    "PROJECT_NAME": f"/{PROJECT_NAME}",
                },
                shell=True,
                capture_output=not verbose,
                # stdout=None if verbose else subprocess.PIPE,  # <-- MODIFIED
                # stderr=None if verbose else subprocess.PIPE,  # <-- MODIFIED
                check=True,
                text=True,
            )
            if verbose:print(result.stdout)
            
        except subprocess.CalledProcessError as e:
            # 捕捉错误并打印错误信息
            print(f"Error: Command '{e.cmd}' failed with return code {e.returncode}")
            print(f"Standard Output:\n{e.stdout}")
            print(f"Standard Error:\n{e.stderr}")
                


    def rm_container(self):
        if self.is_running():
            for op in ["stop", "rm"]:
                cmd = f"docker {op} {self.container_name}"
                subprocess.run(cmd, shell=True, capture_output=True)

    @property
    def container_name(self) -> str:
        return self.ctn_name

    def container_ip(self) -> str:
        """
        Gets the ip address of the container
        :returns: IP address of this container
        """
        assert self.is_running(), f"Container {self.container_name} is not running."
        ctn = docker.from_env().containers.get(self.container_name)
        return ctn.attrs["NetworkSettings"]["IPAddress"]

    def exists(self) -> bool:
        """
        Checks if the container exists
        :returns: True i exists, False otherwise
        """
        try:
            docker.from_env().containers.get(self.container_name)
            return True
        except (docker.errors.NotFound, docker.errors.DockerException):
            return False

    def is_running(self) -> bool:
        """
        Checks if the container is running
        :returns: True if running, False otherwise
        """
        try:
            return (
                docker.from_env().containers.get(self.container_name).status
                == "running"
            )
        except Exception:
            return False

    @property
    def dreamview_url(self) -> str:
        """
        Gets the Dreamview url of the container
        :returns: Dreamview url of this container
        """
        
        return f"http://{self.container_ip()}:8888"
    
    
    # def exec(self, cmd: str, detached=False, verbose=False):
    #     """
    #     Executes a command in the container
    #     :param cmd: Command to execute
    #     :param detached: Whether the command should be executed in detached mode
    #     """
    #     exe = (
    #         f"{DOCKER_CMD} exec "
    #         + "-u $USER "
    #         + f'{"-d " if detached else ""}{self.container_name} {cmd}'
    #     )
        
    #     # 打印执行的命令用于调试
    #     print(f"Executing command: {exe}")

    #     # 执行命令并捕获结果
    #     result = subprocess.run(exe,
    #                             shell=True, 
    #                             capture_output=not verbose,
    #                             # stdout=None if verbose else subprocess.PIPE,  # <-- MODIFIED
    #                             # stderr=None if verbose else subprocess.PIPE,  # <-- MODIFIED
    #                             text=True)
    #     print("----result", result)
    #     # 打印执行结果用于调试
    #     if result.returncode != 0:
    #         print(f"Command failed with return code {result.returncode}")
    #         print(f"Standard Output:\n{result.stdout}")
    #         print(f"Standard Error:\n{result.stderr}")
    #         logger.error(f"Command failed with return code {result.returncode}")
    #         logger.error(f"Standard Output:\n{result.stdout}")
    #         logger.error(f"Standard Error:\n{result.stderr}")
    #     else:
    #         print(f"Command executed successfully: {result.stdout}")
    #         logger.debug(f"Command executed successfully: {result.stdout}")
            

    #     return result
    
    

    # def exec(self, cmd: str, detached=False, verbose=False):
    #     """
    #     Executes a command in the container
    #     :param cmd: Command to execute
    #     :param detached: Whether the command should be executed in detached mode
    #     :param verbose: Whether to capture and log stdout and stderr
    #     """
    #     exe = (
    #         f"{self.DOCKER_CMD} exec "
    #         + "-u $USER "
    #         + f'{"-d " if detached else ""}{self.container_name} {cmd}'
    #     )
        
    #     # 打印执行的命令用于调试
    #     logger.debug(f"Executing command: {exe}")
    #     print(f"Executing command: {exe}")

    #     try:
    #         if detached:
    #             # 使用 subprocess.Popen 进行异步执行
    #             process = subprocess.Popen(
    #                 exe,
    #                 shell=True,
    #                 stdout=subprocess.PIPE if verbose else subprocess.DEVNULL,
    #                 stderr=subprocess.PIPE if verbose else subprocess.DEVNULL,
    #                 text=True
    #             )
                
    #             if verbose:
    #                 # 定义读取输出的函数
    #                 def read_output(pipe, log_func):
    #                     for line in iter(pipe.readline, ''):
    #                         log_func(line.strip())
    #                     pipe.close()
                    
    #                 # 启动线程读取 stdout 和 stderr
    #                 threading.Thread(target=read_output, args=(process.stdout, logger.debug), daemon=True).start()
    #                 threading.Thread(target=read_output, args=(process.stderr, logger.error), daemon=True).start()
                
    #             # 存储子进程对象
    #             self.processes.append(process)
                
    #             # 返回 Popen 对象，以便后续监控（如需要）
    #             return process
    #         else:
    #             # 使用 subprocess.run 进行同步执行
    #             result = subprocess.run(
    #                 exe,
    #                 shell=True, 
    #                 capture_output=not verbose,
    #                 text=True
    #             )
    #             print("----result", result)
    #             # 打印执行结果用于调试
    #             if result.returncode != 0:
    #                 print(f"Command failed with return code {result.returncode}")
    #                 print(f"Standard Output:\n{result.stdout}")
    #                 print(f"Standard Error:\n{result.stderr}")
    #                 logger.error(f"Command failed with return code {result.returncode}")
    #                 logger.error(f"Standard Output:\n{result.stdout}")
    #                 logger.error(f"Standard Error:\n{result.stderr}")
    #             else:
    #                 print(f"Command executed successfully: {result.stdout}")
    #                 logger.debug(f"Command executed successfully: {result.stdout}")
                    
    #             return result
    #     except Exception as e:
    #         logger.error(f"Execution failed: {e}")
    #         print(f"Execution failed: {e}")
    #         raise
    
    def exec_src(self, cmd: str, detached=False, verbose=False):
        """
        Executes a command in the container
        :param cmd: Command to execute
        :param detached: Whether the command should be executed in detached mode
        """
        # 使用 bash -c 来确保 source 命令在 bash 中执行
        exe = (
            f"{DOCKER_CMD} exec "
            + "-u $USER "
            + f'{"-d " if detached else ""}{self.container_name} bash -c "{cmd}"'
        )

        # 打印执行的命令用于调试
        print(f"Executing command: {exe}")

        # 执行命令并捕获结果
        result = subprocess.run(exe,
                                shell=True,
                                capture_output=not verbose,
                                # stdout=None if verbose else subprocess.PIPE,  # <-- MODIFIED
                                # stderr=None if verbose else subprocess.PIPE,  # <-- MODIFIED
                                text=True)

        print("===result",result)
        # 打印执行结果用于调试
        if result.returncode != 0:
            print(f"Command failed with return code {result.returncode}")
            print(f"Standard Output:\n{result.stdout}")
            print(f"Standard Error:\n{result.stderr}")
        else:
            print(f"Command executed successfully: {result.stdout}")

        return result
        
    def exec_python(self, cmd: str, detached=False, verbose=False):
        """
        Executes a command in the container, ensuring the environment is correctly loaded
        :param cmd: Command to execute
        :param detached: Whether the command should be executed in detached mode
        :param verbose: If True, prints full command output
        """
        # 使用 --login 来加载所有登录时的 shell 环境
        exe = (
            f"{DOCKER_CMD} exec "
            + "-u $USER "
            + f'{"-d " if detached else ""}{self.container_name} bash --login -c "{cmd}"'
        )

        # 打印执行的命令用于调试
        print(f"Executing command: {exe}")

        # 执行命令并捕获结果
        result = subprocess.run(exe,
                                shell=True,
                                capture_output=not verbose,
                                # stdout=None if verbose else subprocess.PIPE,  # <-- MODIFIED
                                # stderr=None if verbose else subprocess.PIPE,  # <-- MODIFIED
                                text=True)
        
        # 打印执行结果用于调试
        if result.returncode != 0:
            print(f"Command failed with return code {result.returncode}")
            print(f"Standard Output:\n{result.stdout}")
            print(f"Standard Error:\n{result.stderr}")
        else:
            print(f"Command executed successfully: {result.stdout}")

        return result
    
    
    def start_dreamview(self):
        self.exec("bash /apollo/scripts/dreamview.sh start")

    def restart_dreamview(self):
        self.exec("bash /apollo/scripts/dreamview.sh restart")

    def start_bridge(self):
        self.exec("bash /apollo/scripts/bridge.sh", detached=True)

    def start_planning(self):
        self.exec("bash /apollo/scripts/planning.sh start")

    def start_routing(self):
        self.exec("bash /apollo/scripts/routing.sh start")

    def start_prediction(self):
        self.exec("bash /apollo/scripts/prediction.sh start")
        
        
    ###Wei customized
    def install_bridge_lib(self):
        self.exec_src("source launch_carla_bridge.sh")

        
    def start_camera(self):
        self.exec("bash /apollo/scripts/camera.sh start")

    def start_canbus(self):
        self.exec("bash /apollo/scripts/canbus.sh start")

    def start_control(self):
        self.exec("bash /apollo/scripts/control.sh start")

    def start_GPS(self):
        self.exec("bash /apollo/scripts/gps.sh start")

    def start_guardian(self):
        self.exec("bash /apollo/scripts/guardian.sh start")

    def start_localization(self):
        self.exec("bash /apollo/scripts/localization.sh start")

    def start_perception(self):
        self.exec("bash /apollo/scripts/perception.sh start")

    # def start_traffic_light(self):
    #     self.exec("bash /apollo/scripts/traffic_light.sh start")

    def start_transform(self):
        self.exec("bash /apollo/scripts/transform.sh start")

    def start_velodyne(self):
        self.exec("bash /apollo/scripts/velodyne.sh start")

    #TODO: 看情况是直接在这些函数里加sleep 还是在调用的时候加sleep
    def start_carla(self):
        run_command(CARLA_SIMULATOR['path'], CARLA_SIMULATOR['args'], cwd=os.path.dirname(CARLA_SIMULATOR['path']))







    def exec(self, cmd: str, detached=False, verbose=False):
        """
        Executes a command in the container
        :param cmd: Command to execute
        :param detached: Whether the command should be executed in detached mode
        :param verbose: Whether to capture and log stdout and stderr
        """
        exe = (
            f"{DOCKER_CMD} exec "
            + "-u $USER "
            + f'{"-d " if detached else ""}{self.container_name} {cmd}'
        )
        
        # 打印执行的命令用于调试
        logger.info(f"Executing command: {exe}")
        print(f"Executing command: {exe}")

        try:
            if detached:
                # 使用 subprocess.Popen 进行异步执行
                process = subprocess.Popen(
                    exe,
                    shell=True,
                    stdout=subprocess.PIPE if verbose else subprocess.DEVNULL,
                    stderr=subprocess.PIPE if verbose else subprocess.DEVNULL,
                    text=True
                )
                
                if verbose:
                    stdout, stderr = process.communicate()
                    if stdout:
                        logger.debug(f"Command stdout: {stdout}")
                        print(f"Command stdout: {stdout}")
                    if stderr:
                        logger.error(f"Command stderr: {stderr}")
                        print(f"Command stderr: {stderr}")
                
                # 返回 Popen 对象，以便后续监控（如需要）
                return process
            else:
                # 使用 subprocess.run 进行同步执行
                result = subprocess.run(
                    exe,
                    shell=True, 
                    capture_output=not verbose,
                    text=True
                )
                print("----result", result)
                # 打印执行结果用于调试
                if result.returncode != 0:
                    print(f"Command failed with return code {result.returncode}")
                    print(f"Standard Output:\n{result.stdout}")
                    print(f"Standard Error:\n{result.stderr}")
                    logger.error(f"Command failed with return code {result.returncode}")
                    logger.error(f"Standard Output:\n{result.stdout}")
                    logger.error(f"Standard Error:\n{result.stderr}")
                else:
                    print(f"Command executed successfully: {result.stdout}")
                    logger.debug(f"Command executed successfully: {result.stdout}")
                        
                return result
        except Exception as e:
            logger.error(f"Execution failed: {e}")
            print(f"Execution failed: {e}")
            raise

    def launch_bridge(self, replay=False, replay_file=None, objects_file=None,replay_log=None,num_vehicles=None, num_walkers=None,
                      record_path=None, record_replay_path=None, metric_dir=None):
        """
        在容器内启动 Carla bridge
        docker exec apollo_dev_w ps aux | grep "python3.*main.py" | grep -v grep

        # 然后用 ps -T 查看该进程的所有线程
        docker exec apollo_dev_w ps -T -p <PID>
        """
        
        print("****************************************")
        print("LAUNCH BRIDGE")
        print("replay is:",replay,"objects_file is:",objects_file)
        print("**************************************************")
                # 1. 首先检查环境
        # if not self.check_environment_variables():
        #     raise RuntimeError("Environment variables not properly set")
            
        # # 2. 检查Cyber框架
        # if not self.check_cyber_status():
        #     raise RuntimeError("Cyber framework not properly initialized")
            
        try:
            logger.info("Starting Carla bridge...")
            
            pythonpath = (
                "/apollo/bazel-bin/cyber/python/internal:"
                "/apollo/cyber:"
                "/apollo/cyber/python:"
                "/apollo:"
                "/apollo/modules:"
                "/apollo/modules/carla_bridge/carla_api/carla-0.9.14-py3.7-linux-x86_64.egg:"
                "/apollo/bazel-bin"
            )
            
            # 构建命令
            cmd = (
                f"cd /apollo/modules/carla_bridge && "
                f"export PYTHONPATH={pythonpath} && "
                "python3 main.py"
            )
            
            
            if replay:
                if not replay_file:
                    raise ValueError("replay_file must be provided when replay is True")
                

                cmd += f" --replay --replay_file {self.convert_host_path_to_container(replay_file)}"
                
                
            if objects_file:
                cmd += f" --objects_file {self.convert_host_path_to_container(objects_file)}"
            
            
            if replay_log:
                cmd+=f" --replay_log {self.convert_host_path_to_container(replay_log)}"
            
            if num_vehicles:
                cmd+=f" --num_vehicles {num_vehicles}"
            

            if num_walkers:
                cmd+=f" --num_walkers {num_walkers}"
            
            if record_path:
                cmd+=f" --record_path {self.convert_host_path_to_container(record_path)}"
            
            
            if record_replay_path:
                cmd+=f" --record_replay_path {self.convert_host_path_to_container(record_replay_path)}"
            
            
            if metric_dir:
                cmd+=f" --metric_dir {self.convert_host_path_to_container(metric_dir)}"
            # 打印命令用于调试
            logger.debug(f"Generated command: {cmd}")
            print(f"Generated command: {cmd}")
            
            # 使用当前用户执行命令
            process = self.exec(f'bash -c "{cmd}"',  detached=True, verbose=True)
            
            time.sleep(2)
 
            # 在异步模式下，process 是一个 Popen 对象
            if isinstance(process, subprocess.Popen):
                logger.info("Successfully initiated launching Carla bridge asynchronously")
                print("Successfully initiated launching Carla bridge asynchronously")
                bridge_process_name = "main.py"
                
                # 等待桥接进程启动
                bridge_up = False
                timeout = 30
                interval = 1
                elapsed = 0
                
                while elapsed < timeout:
                    if self.is_process_running(bridge_process_name):
                        bridge_up = True
                        logger.info("Carla bridge is running.")
                        print("Carla bridge is running.")
                        break
                    time.sleep(interval)
                    elapsed += interval
                    logger.debug(f"Waiting for Carla bridge to start... {elapsed}s elapsed.")
                    print(f"Waiting for Carla bridge to start... {elapsed}s elapsed.")
                
                if not bridge_up:
                    logger.error("Carla bridge failed to start within the timeout period.")
                    print("Carla bridge failed to start within the timeout period.")
                    return False
            else:
                # 在同步模式下，process 是一个 CompletedProcess 对象
                if process.returncode != 0:
                    logger.error(f"Failed to launch bridge: {process.stderr}")
                    logger.error(f"Command output: {process.stdout}")
                    return False
                else:
                    logger.info("Successfully launched bridge")
                    print("Successfully launched bridge")
            
            return True
            
        except Exception as e:
            logger.error(f"Error launching bridge: {str(e)}")
            print(f"Error launching bridge: {str(e)}")
            return False

    
    
    # def carla_record_data(self, output_path: str):
    #     """
    #     执行数据记录脚本
    #     :param output_path: 指定输出文件的路径
    #     """
    #     script_path = str(Path(__file__).parent / 'record_data.py')
    #     args = [script_path, '--output', output_path]
        
    #     run_command("python3", args, cwd=os.path.dirname(script_path))




    """
    def generate_traffic(self, num_vehicles: int = 20, num_walkers: int = 20, 
                        safe: bool = True, sync: bool = False):

        try:
            logger.info(f"Generating traffic with {num_vehicles} vehicles and {num_walkers} walkers")
            
            # 定义 PYTHONPATH 的值
            python_path = (
                "/apollo/bazel-bin/cyber/python/internal:"
                "/apollo/cyber/python:"
                "/apollo:"
                "/apollo/modules:"
                "/apollo/modules/carla_bridge/carla_api/carla-0.9.14-py3.7-linux-x86_64.egg:"
                "/apollo/bazel-bin"
            )
            
            # 构建命令，使用双引号括起 PYTHONPATH 的值，并移除 & 符号
            cmd = (
                "cd /apollo/modules/carla_bridge && "
                f'export PYTHONPATH="{python_path}" && '
                f"python3 /apollo/modules/carla_bridge/multi_vehicle_fuzz/generate_traffic.py "
                f"-n {num_vehicles} "
                f"-w {num_walkers} "
                f"{'--safe ' if safe else ''}"
                f"{'--sync' if sync else '--async'}"
            )

            # 打印命令用于调试
            logger.debug(f"Generated command: {cmd}")
            
            # 使用 self.exec 执行命令，设置 detached=True 以异步执行
            result = self.exec(f"bash -c '{cmd}'", detached=True, verbose=True)

            # 由于是异步执行，立即返回
            logger.info("Successfully initiated traffic generation")
            return True
                
        except Exception as e:
            logger.error(f"Failed to generate traffic: {e}")
            raise
    """ 
        
        
            

    def generate_traffic(self, num_vehicles: int = 20, num_walkers: int = 20, 
                        safe: bool = True, sync: bool = False):
        """
        在容器外部生成交通场景

        Args:
            num_vehicles (int): 车辆数量
            num_walkers (int): 行人数量
            safe (bool): 是否启用安全模式
            sync (bool): 是否使用同步模式
        """
        try:
            logger.info(f"Generating traffic with {num_vehicles} vehicles and {num_walkers} walkers (outside container)")

            # 构建命令
            cmd = [
                "python3",
                str(Path(PROJECT_ROOT) /"generate_traffic.py"),
                "-n", str(num_vehicles),
                "-w", str(num_walkers),
            ]
            
            # 添加安全模式和同步模式选项
            if safe:
                cmd.append("--safe")
            cmd.append("--sync" if sync else "--async")

            logger.info(f"Executing command: {' '.join(cmd)}")

            # 使用 Popen 启动子进程
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,  # 将标准输出重定向到主进程
                stderr=subprocess.PIPE,  # 将错误输出重定向到主进程
                cwd=str(PROJECT_ROOT)  # 设置工作目录
            )

            # 将进程对象保存到 self.processes 中，以便后续管理
            self.processes.append(process)

            # 验证进程是否正常启动
            time.sleep(2)  # 给进程一些时间进行启动
            if process.poll() is not None:  # 如果返回值不是 None，说明进程已退出
                out, err = process.communicate()
                logger.error(f"Traffic generation process failed to start: {err.decode('utf-8')}")
                return False

            logger.info(f"Started traffic generation process with PID {process.pid}")
            return True

        except Exception as e:
            logger.error(f"Failed to generate traffic: {e}")
            logger.exception("Detailed error:")
            return False   
            
            
        
        

    def start_ads_modules(self):
        print("******************************************************")
        print("***************START_ ADS MODULE***********************")
        print("******************************************************")
        #FIXED: 之前模块启动不起来就是因为bootstarp没有被运行
        self.exec("bash scripts/bootstrap.sh")
        #FIXME:  这里bridge 需要被执行吗？
        self.start_routing()
        self.start_prediction()
        self.start_planning()

        self.start_camera()
        self.start_canbus()
        self.start_control()
        self.start_GPS()
        self.start_guardian()
        self.start_localization()
        self.start_perception()
        # self.start_traffic_light()
        self.start_transform()
        self.start_velodyne()
        #TODO: 看下要不要启动recorder 还是说一个额外的
        
    def stop_planning_n_prediction(self):
        cmd = "pkill --signal SIGKILL -f 'planning|prediction|control'"
        self.exec(cmd)


    # def stop_ads_modules(self):
    #     cmd = "pkill --signal SIGINT -f 'planning|routing|prediction|cyber_bridge'"
    #     self.exec(cmd)
    def stop_ads_modules(self):
        """停止所有 ADS 相关模块"""
        cmd = "pkill --signal SIGINT -f 'planning|routing|prediction|cyber_bridge|" \
              "camera|canbus|control|gps|guardian|localization|perception|" \
              "transform|velodyne'"
        self.exec(cmd)
    def start_sim_control(self, x: float, y: float, heading: float):
        executable = "/apollo/bazel-bin/modules/sim_control_standalone/main"
        cmd = f"{executable} {x} {y} {heading}"
        self.exec(cmd, detached=True)

    def stop_sim_control(self):
        cmd = "pkill --signal SIGINT -f 'sim_control_standalone'"
        self.exec(cmd)

    def start_replay(self, filename: str):
        # cyber_recorder play -f <file>
        # TODO: add --log_dir=/apollo/data/log
        cyber_recorder = "/apollo/bazel-bin/cyber/tools/cyber_recorder/cyber_recorder"
        cmd = f"{cyber_recorder} play -f {filename}"
        self.exec(cmd, detached=True)

    def stop_replay(self):
        cmd = "pkill --signal SIGINT -f 'cyber_recorder play'"
        self.exec(cmd)

    #FIXME: 这个函数实际上没被用到，后期应该清理掉
    def start_recorder(self, filename: str):
        # cyber_recorder record -o <file>
        # TODO: add --log_dir=/apollo/data/log
        cyber_recorder = "/apollo/bazel-bin/cyber/tools/cyber_recorder/cyber_recorder"
        cmd = f"{cyber_recorder} record -a -o {filename}"
        self.exec(cmd, detached=True)

    def stop_recorder(self):
        try:
            cmd = "pkill --signal SIGINT -f 'cyber_recorder record'"
            self.exec(cmd)
            # time.sleep(1)  # 给进程一些终止的时间
        except Exception as e:
            logger.error(f"Failed to stop recorder: {e}")

    def stop_apollo(self):
        cmd="pkill --signall SIGINT -f 'python'" 
        self.exec(cmd)
            
    def stop_carla(self):
        """停止 Carla 进程"""
        process = run_command('pkill', ['-9', 'CarlaUE4'])
        
        if process is not None:
            stdout, stderr = process.communicate()  # 等待进程完成并获取输出
            logger.debug(f"Carla stop command output: {stdout.decode()}")
            if stderr:
                logger.warning(f"Carla stop command error: {stderr.decode()}")
    
    def stop_bridge(self):
        """停止cyber_bridge进程"""
        print("==========================================================")
        print("========== stop Bridge in container =======")
        print("==========================================================")
        
        try:
            logger.info(f"Stopping bridge in container {self.container_name}")
            # 使用pkill命令终止cyber_bridge进程
            self.exec("pkill --signal SIGINT -f 'python3.*main.py'")
            self.exec("pkill --signal SIGINT -f 'python.*main.py'")
            
            # 或者更精确地：
            # self.exec("pkill --signal SIGINT -f '/apollo/modules/carla_bridge/main.py'")
            time.sleep(2)  # 给进程一些终止的时间
            logger.info("Bridge stopped successfully")
        except Exception as e:
            logger.error(f"Failed to stop bridge: {e}")
            raise
        
    def stop_routing(self):
        """停止 routing_handler 进程"""
        try:
            logger.info("Stopping routing handler...")
            self.exec("pkill --signal SIGINT -f 'routing_handler.py'")
            time.sleep(2)

            # 检查进程是否仍在运行
            if self.is_process_running("routing_handler.py"):
                logger.warning("Routing handler still running after SIGINT, forcing termination...")

                # 强制杀死进程
                self.exec("pkill -9 -f 'routing_handler.py'")
                time.sleep(1)
            else:
                logger.info("Routing handler stopped successfully.")

        except Exception as e:
            logger.error(f"Error stopping routing handler: {e}")
        
    # def stop_carla_record(self):
    #     """停止Carla数据记录进程"""
    #     try:
    #         logger.info("Stopping Carla record process")
    #         # 使用pkill命令终止carla_record_data进程
    #         self.exec("pkill --signal SIGINT -f 'python3.*carla_record_data.py'")
    #         # 或者更精确地：
    #         # self.exec("pkill --signal SIGINT -f '/apollo/modules/tools/carla_record_data.py'")
    #         time.sleep(1)  # 给进程一些终止的时间
    #         logger.info("Carla record process stopped successfully")
    #     except Exception as e:
    #         logger.error(f"Failed to stop Carla record process: {e}")
    #         raise
        
        
    # def carla_record_data(self, output_path: str, carla_record_path: str = None):
    #     """
    #     执行数据记录脚本
    #     :param output_path: 指定JSON格式的车辆状态输出文件路径
    #     :param carla_record_path: 指定Carla原生录制文件的路径
    #     """
    #     try:
    #         script_path = str(Path(__file__).parent / 'record_data.py')
    #         args = [script_path, '--output', output_path]
            
    #         if carla_record_path:
    #             args.extend(['--carla-record', carla_record_path])
            
    #         logger.info(f"Starting data recording:")
    #         logger.info(f"- Vehicle states: {output_path}")
    #         if carla_record_path:
    #             logger.info(f"- Carla recorder: {carla_record_path}")
            
    #         run_command("python3", args, cwd=os.path.dirname(script_path))
            
    #     except Exception as e:
    #         logger.error(f"Failed to start data recording: {e}")
    #         raise
        
            
    # def _verify_record_data(self, output_path: str, max_retries: int = 10, retry_interval: int = 2) -> bool:
    #     """
    #     验证记录数据进程是否正常运行
        
    #     Args:
    #         output_path: 输出文件路径
    #         max_retries: 最大重试次数
    #         retry_interval: 重试间隔（秒）
        
    #     Returns:
    #         bool: 验证是否成功
    #     """
    #     try:
    #         output_dir = os.path.dirname(output_path)
            
    #         # 确保输出目录存在
    #         if not os.path.exists(output_dir):
    #             os.makedirs(output_dir)
    #             logger.info(f"Created output directory: {output_dir}")
            
    #         # 检查进程是否成功启动并运行
    #         for i in range(max_retries):
    #             if self.is_process_running('record_data.py'):
    #                 logger.info("Record data process is running")
    #                 return True
    #             else:
    #                 logger.warning(f"Waiting for record data process to start (attempt {i+1}/{max_retries})")
    #                 time.sleep(retry_interval)
            
    #         logger.error("Failed to verify record data process")
    #         return False
                
    #     except Exception as e:
    #         logger.error(f"Error verifying record data process: {e}")
    #         return False        
            
    # def carla_record_data(self, output_path: str, carla_record_path: str = None):
    #     """
    #     启动数据记录进程
        
    #     Args:
    #         output_path: JSON输出文件路径
    #         carla_record_path: Carla记录文件路径（可选）
        
    #     Returns:
    #         bool: 是否成功启动记录
    #     """
    #     print("===========================call carla record data")
    #     print("===========================output path is:", output_path)
        
    #     try:
    #         # 构建命令
    #         cmd = f"""source /apollo/cyber/setup.bash && \
    #             source /apollo/scripts/apollo_base.sh && \
    #             export PYTHONPATH=/apollo/bazel-bin/cyber/python/internal:\
    #             /apollo/cyber:\
    #             /apollo/cyber/python:\
    #             /apollo:\
    #             /apollo/modules:\
    #             /apollo/modules/carla_bridge/carla_api/carla-0.9.14-py3.7-linux-x86_64.egg:\
    #             /apollo/bazel-bin:\
    #             /home/w/.local/lib/python3.6/site-packages && \
    #             cd /apollo/modules/carla_bridge && \
    #             python3 multi_vehicle_fuzz/record_data.py --output {output_path}"""
                
    #         if carla_record_path:
    #             cmd += f" --carla-record {carla_record_path}"
                
    #         # 启动记录进程
    #         process = self.exec(f'bash -c "{cmd}"', detached=True)
            
    #         # 验证进程是否正常运行
    #         if not self._verify_record_data(output_path):
    #             logger.error("Failed to start record data process")
    #             return False
                
    #         logger.info("Successfully started record data process")
    #         return True
            
    #     except Exception as e:
    #         logger.error(f"Failed to start record data: {e}")
    #         return False
   
   
    def carla_record_data(self, output_path: str, carla_record_path: str = None):
        """在宿主机上启动数据记录进程"""
        try:
            logger.info("Starting record data with paths:")
            logger.info(f"Output path: {output_path}")
            logger.info(f"Carla record path: {carla_record_path}")
            
            # 构建命令
            cmd = [
                "python3",
                "-u",  # 添加-u参数，禁用Python输出缓冲
                str(Path(PROJECT_ROOT) / "record_data.py"),
                "--output", str(output_path)
            ]
            
            if carla_record_path:
                cmd.extend(["--carla-record", str(carla_record_path)])
                
            logger.info(f"Executing command: {' '.join(cmd)}")
            
            # 直接使用Popen而不是run_command
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                bufsize=1,  # 行缓冲
                cwd=str(PROJECT_ROOT)
            )
        
            # 创建线程来实时读取输出
            def log_output(pipe, prefix):
                try:
                    for line in pipe:
                        logger.info(f"[Record Data {prefix}] {line.strip()}")
                except Exception as e:
                    logger.error(f"Error reading {prefix} from record_data.py: {e}")
            
            # 启动输出监控线程
            stdout_thread = threading.Thread(target=log_output, args=(process.stdout, "OUT"), daemon=True)
            stderr_thread = threading.Thread(target=log_output, args=(process.stderr, "ERR"), daemon=True)
            
            stdout_thread.start()
            stderr_thread.start()
            
            logger.info(f"Started record data process with PID {process.pid}")
            self.processes.append(process)
            
            # 验证进程是否正常启动
            time.sleep(2)  # 给进程一些启动时间
            if process.poll() is not None:
                logger.error("Record data process failed to start")
                return False
                
            return True
            
        except Exception as e:
            logger.error(f"Error starting record data: {e}")
            logger.exception("Detailed error:")
            return False
        
        
        
        
    def carla_recorder(self, output_path: str):
        """启动 CARLA 日志记录进程（非阻塞）"""
        try:
            logger.info(f"Carla recorder output path is: {output_path}")
            
            # 构建命令
            cmd = [
                "python3",
                str(Path(PROJECT_ROOT) / "carla_log_recorder.py"),
                "--output_path", str(output_path)
            ]
            
            logger.info(f"Executing command: {' '.join(cmd)}")
            
            # 直接使用 Popen 启动子进程
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,  # 不需要输出日志到主进程
                stderr=subprocess.DEVNULL,  # 不需要错误日志到主进程
                cwd=str(PROJECT_ROOT)  # 设置工作目录
            )
            
            # 将进程对象保存到 self.processes 中，以便后续管理
            self.processes.append(process)
            
            # 验证进程是否正常启动
            time.sleep(2)  # 给进程一些时间进行启动
            if process.poll() is not None:  # 如果返回值不是 None，说明进程已退出
                logger.error("Carla recorder process failed to start")
                return False

            logger.info(f"Started Carla recorder process with PID {process.pid}")
            return True

        except Exception as e:
            logger.error(f"Error starting Carla recorder: {e}")
            logger.exception("Detailed error:")
            return False
        
    def stop_carla_recorder(self):
        """停止 CARLA 日志记录进程"""
        print("==========================================================")
        print("========== Stop CARLA recorder in container =======")
        print("==========================================================")
        try:
            logger.info("Stopping CARLA recorder process")

            # 遍历 self.processes，找到与 CARLA recorder 相关的进程并发送 SIGTERM
            for process in self.processes:
                if process.poll() is None:  # 如果进程仍在运行
                    logger.info(f"Sending SIGTERM to process with PID {process.pid}")
                    process.terminate()  # 发送 SIGTERM 信号

            # 等待所有子进程优雅退出
            time.sleep(2)  # 给子进程一些清理时间
            
            # 再次检查进程状态，确保所有进程都已退出
            for process in self.processes:
                if process.poll() is None:  # 如果进程仍然在运行
                    logger.warning(f"Process with PID {process.pid} did not terminate, sending SIGKILL")
                    process.kill()  # 强制终止进程

            # 清理 self.processes 列表
            self.processes = [p for p in self.processes if p.poll() is None]

            logger.info("CARLA recorder process stopped")
        except Exception as e:
            logger.error(f"Failed to stop CARLA recorder process: {e}")
            logger.exception("Detailed error:")
    
    def record_replay_data(self, output_path: str, log_dir: str, replay_index: str):
        """在容器内启动回放数据记录进程"""
        print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        print("++++++++++++Call RECORD REPLAY DATA++++++++++++++++")
        print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        
        try:
            logger.info("Starting replay record data with paths:")
            
            # 转换路径到容器内路径
            container_output_path = self.convert_host_path_to_container(output_path)
            container_log_dir = self.convert_host_path_to_container(log_dir)
            # carla_record_path = os.path.join(container_log_dir, "carla_record_new.log")
            
            
            
            logger.info(f"Container output path: {container_output_path}")
            logger.info(f"Container log directory: {container_log_dir}")
            logger.info(f"Replay index: {replay_index}")
            
            # 构建 PYTHONPATH
            pythonpath = (
                "/apollo/bazel-bin/cyber/python/internal:"
                "/apollo/cyber:"
                "/apollo/cyber/python:"
                "/apollo:"
                "/apollo/modules:"
                "/apollo/modules/carla_bridge/carla_api/carla-0.9.14-py3.7-linux-x86_64.egg:"
                "/apollo/bazel-bin"
            )
            
            # 构建容器内命令
            cmd = (
                f"cd /apollo/modules/carla_bridge/multi_vehicle_fuzz && "
                f"export PYTHONPATH={pythonpath} && "
                f"python3 -u record_replay_data.py "
                f"--output \"{container_output_path}\" "
                f"--log-dir \"{container_log_dir}\" "
                f"--replay-index \"{replay_index}\""
                
            )
            
            logger.debug(f"Generated command: {cmd}")
            
            # 使用 `exec` 函数以当前用户执行命令，启用 detached 模式运行
            process = self.exec(f"bash -c \"{cmd}\"", detached=True, verbose=True)
            
            # 等待数据回放进程启动
            time.sleep(2)
            
            # 检查 `record_replay_data.py` 是否已启动
            if not self.is_process_running("record_replay_data.py"):
                logger.error("Replay record process failed to start")
                return False
            
            logger.info("Replay record process started successfully")
            return True

        except Exception as e:
            logger.error(f"Error starting replay record: {e}")
            logger.exception("Detailed error:")
            return False
        
        
    def stop_record_replay_data(self):
        """正常停止record_replay_data.py进程"""
        print("==========================================================")
        print("========== Stop record replay data in container =======")
        print("==========================================================")
        
        try:
            logger.info(f"Stopping record replay data in container {self.container_name}")
            
            # 使用pkill命令发送SIGINT信号终止record_replay_data.py进程
            self.exec("pkill --signal SIGINT -f 'python3.*record_replay_data.py'")
            
            # 给进程一些终止的时间
            time.sleep(2)  
            
            # 检查进程是否仍在运行
            if self.is_process_running("record_replay_data.py"):
                logger.warning("Record replay data still running after SIGINT, forcing termination...")
                # 如果进程还在运行，再尝试一次SIGINT
                self.exec("pkill --signal SIGINT -f 'python.*record_replay_data.py'")
                time.sleep(1)
            
            logger.info("Record replay data stopped successfully")
            
        except Exception as e:
            logger.error(f"Failed to stop record replay data: {e}")
            raise
            
            
        
        
        
    # def record_replay_data(self, output_path: str, log_dir: str, replay_index: str):
    #     """在宿主机上启动回放数据记录进程"""
    #     try:
    #         logger.info("Starting replay record data with paths:")
    #         logger.info(f"Output path: {output_path}")
    #         logger.info(f"Log directory: {log_dir}")
    #         logger.info(f"Replay index: {replay_index}")
            
    #         # 构建命令
    #         # FIXME: 这里前面出问题是因为叠加了两个multi_vehicle_fuzz 的路径，删除之后再测试一下
    #         cmd = [
    #             "python3",
    #             "-u",  # 禁用Python输出缓冲 
    #             str(Path(PROJECT_ROOT) / "record_replay_data.py"), 
    #             "--output", str(output_path),
    #             "--log-dir", str(log_dir),
    #             "--replay-index", replay_index
    #         ]
                
    #         logger.info(f"Executing command: {' '.join(cmd)}")
            
    #         # 启动进程
    #         process = subprocess.Popen(
    #             cmd,
    #             stdout=subprocess.PIPE,
    #             stderr=subprocess.PIPE,
    #             universal_newlines=True,
    #             bufsize=1,  # 行缓冲
    #             cwd=str(PROJECT_ROOT)
    #         )
        
    #         # 创建线程来实时读取输出
    #         def log_output(pipe, prefix):
    #             try:
    #                 for line in pipe:
    #                     logger.info(f"[Replay Record {prefix}] {line.strip()}")
    #             except Exception as e:
    #                 logger.error(f"Error reading {prefix} from record_replay_data.py: {e}")
            
    #         # 启动输出监控线程
    #         stdout_thread = threading.Thread(target=log_output, args=(process.stdout, "OUT"), daemon=True)
    #         stderr_thread = threading.Thread(target=log_output, args=(process.stderr, "ERR"), daemon=True)
            
    #         stdout_thread.start()
    #         stderr_thread.start()
            
    #         logger.info(f"Started replay record process with PID {process.pid}")
    #         self.processes.append(process)
            
    #         # 验证进程是否正常启动
    #         time.sleep(2)
    #         if process.poll() is not None:
    #             logger.error("Replay record process failed to start")
    #             return False
                
    #         return True
            
    #     except Exception as e:
    #         logger.error(f"Error starting replay record: {e}")
    #         logger.exception("Detailed error:")
    #         return False
    
    
    
    # def carla_record_data(self, output_path: str, carla_record_path: str = None):
    #     """在宿主机上启动数据记录进程"""
    #     try:
    #         # 输出详细的路径信息
    #         logger.info("Starting record data with paths:")
    #         logger.info(f"Output path: {output_path}")
    #         logger.info(f"Absolute output path: {os.path.abspath(output_path)}")
    #         if carla_record_path:
    #             logger.info(f"Carla record path: {carla_record_path}")
    #             logger.info(f"Absolute carla record path: {os.path.abspath(carla_record_path)}")
            
    #         # 检查目录是否存在
    #         output_dir = os.path.dirname(output_path)
    #         logger.info(f"Output directory: {output_dir}")
    #         if not os.path.exists(output_dir):
    #             logger.warning(f"Output directory does not exist, creating: {output_dir}")
    #             os.makedirs(output_dir, exist_ok=True)

    #         # 构建命令
    #         cmd = [
    #             "python3",
    #             str(Path(PROJECT_ROOT) / "multi_vehicle_fuzz" / "record_data.py"),
    #             "--output", str(output_path)
    #         ]
            
    #         if carla_record_path:
    #             cmd.extend(["--carla-record", str(carla_record_path)])
                
    #         # 输出完整命令
    #         logger.info(f"Executing command: {' '.join(cmd)}")
            
    #         # 在宿主机上启动进程
    #         process = run_command(
    #             cmd[0],  # python3
    #             cmd[1:],  # 其余参数
    #             cwd=str(PROJECT_ROOT)
    #         )
            
    #         if process:
    #             self.processes.append(process)
    #             logger.info(f"Started record data process with PID {process.pid}")
    #             return True
                
    #         logger.error("Failed to start record data process")
    #         return False
            
    #     except Exception as e:
    #         logger.error(f"Error starting record data: {e}")
    #         logger.exception("Detailed error:")  # 这会输出完整的错误堆栈
    #         return False
        
    
    def _verify_record_data(self, output_path: str, max_retries: int = 10, retry_interval: int = 2) -> bool:
        """验证记录数据进程是否正常运行"""
        try:
            output_dir = os.path.dirname(output_path)
            os.makedirs(output_dir, exist_ok=True)
            
            # 检查进程是否成功启动并运行
            for i in range(max_retries):
                # 在宿主机上检查进程
                result = subprocess.run(
                    ["pgrep", "-f", "record_data.py"],
                    capture_output=True,
                    text=True
                )
                
                if result.returncode == 0:
                    logger.info("Record data process is running")
                    return True
                else:
                    logger.warning(f"Waiting for record data process to start (attempt {i+1}/{max_retries})")
                    time.sleep(retry_interval)
            
            logger.error("Failed to verify record data process")
            return False
                
        except Exception as e:
            logger.error(f"Error verifying record data process: {e}")
            return False

    def stop_carla_record(self):
        print("==========================================================")
        print("========== stop carla recorder in container =======")
        print("==========================================================")
        
        """停止数据记录进程"""
        try:
            logger.info("Stopping record data process")
            # 在宿主机上终止进程
            subprocess.run(
                ["pkill", "--signal", "SIGINT", "-f", "record_data.py"],
                check=False
            )
            time.sleep(1)  # 给进程一些清理时间
            
            # 确保进程已经终止
            if subprocess.run(["pgrep", "-f", "record_data.py"], capture_output=True).returncode == 0:
                logger.warning("Process still running after SIGINT, using SIGKILL")
                subprocess.run(["pkill", "-9", "-f", "record_data.py"], check=False)
                
            logger.info("Record data process stopped")
        except Exception as e:
            logger.error(f"Failed to stop record data process: {e}")
   
   
    # def stop_traffic(self):
    #     """停止交通流生成进程"""
    #     try:
    #         logger.info("Stopping traffic generation...")
            
    #         # 首先尝试使用 SIGINT 优雅地停止
    #         self.exec("pkill --signal SIGINT -f 'python3.*generate_traffic.py'")
    #         time.sleep(3)  # 给一些时间让清理过程执行
            
    #         # 检查进程是否还在运行
    #         if self.is_process_running("generate_traffic.py"):
    #             logger.warning("Traffic generation still running after SIGINT, forcing termination...")
    #             # 如果进程还在，使用 SIGKILL 强制终止
    #             self.exec("pkill -9 -f 'python3.*generate_traffic.py'")
                
    #         # 额外检查是否有遗留的 traffic manager 进程
    #         if self.is_process_running("traffic_manager"):
    #             logger.info("Cleaning up traffic manager processes...")
    #             self.exec("pkill -9 -f traffic_manager")
                
    #         logger.info("Traffic generation stopped successfully")
            
    #     except Exception as e:
    #         logger.error(f"Error stopping traffic generation: {e}")
   

    def stop_traffic(self):
        """停止交通流生成进程"""
        print("==========================================================")
        print("========== Stopping traffic generation process ==========")
        print("==========================================================")
        
        try:
            logger.info("Stopping traffic generation process...")

            # 尝试优雅停止 `generate_traffic.py` 进程
            subprocess.run(
                ["pkill", "--signal", "SIGINT", "-f", "generate_traffic.py"],
                check=False  # 即使命令失败也不抛出异常
            )
            time.sleep(1)  # 给进程一些时间进行清理

            # 检查是否仍有 `generate_traffic.py` 进程在运行
            if subprocess.run(["pgrep", "-f", "generate_traffic.py"], capture_output=True).returncode == 0:
                logger.warning("Traffic generation process still running after SIGINT, using SIGKILL")
                # 使用 SIGKILL 强制终止
                subprocess.run(["pkill", "-9", "-f", "generate_traffic.py"], check=False)

            # 确保 `traffic_manager` 相关进程也被清理
            if subprocess.run(["pgrep", "-f", "traffic_manager"], capture_output=True).returncode == 0:
                logger.info("Cleaning up traffic manager processes...")
                subprocess.run(["pkill", "-9", "-f", "traffic_manager"], check=False)

            logger.info("Traffic generation process stopped successfully")

        except Exception as e:
            logger.error(f"Failed to stop traffic generation process: {e}")
   
    def set_destination(self,mode,test_dir=None):
        print("***************SET DESTINATION************************")
        """在容器内部设置随机目的地并返回路由请求"""
        try:
            # 构建完整的 PYTHONPATH
            pythonpath = "/apollo/bazel-bin/cyber/python/internal:" \
                        "/apollo/cyber:" \
                        "/apollo/cyber/python:" \
                        "/apollo:" \
                        "/apollo/modules:" \
                        "/apollo/modules/carla_bridge/carla_api/carla-0.9.14-py3.7-linux-x86_64.egg:" \
                        "/apollo/bazel-bin:" \
                        "/apollo/modules/carla_bridge"
            
            # 构建命令，与原始 docker exec 命令保持一致
            cmd = (
                "source /apollo/cyber/setup.bash && "
                "source /apollo/scripts/apollo_base.sh && "
                f'export PYTHONPATH=$PYTHONPATH:{pythonpath} && '
                "cd /apollo/modules/carla_bridge/multi_vehicle_fuzz && "
                f"python3 routing_handler.py --mode {mode}"
            )
            if test_dir:
                cmd += f" --test_dir {self.convert_host_path_to_container(test_dir)}"
            # 打印命令用于调试
            cmd += " && exit 0"
            logger.info(f"Set-Destination: Generated command: {cmd}")
            print(f"Set-Destination: Generated command: {cmd}")
            
            # 使用 self.exec 执行命令，设置 detached=True 以异步执行
            process = self.exec(f"bash -c '{cmd}'", detached=True, verbose=True)
            
            # 在异步模式下，process 是一个 Popen 对象
            if isinstance(process, subprocess.Popen):
                logger.info("Successfully initiated setting destination asynchronously")
                print("Successfully initiated setting destination asynchronously")
                # Optionally, you can monitor the process or handle cleanup if needed
                # 例如，将 process 对象存储以便稍后终止
                
                # 记录进程ID
                routing_handler_pid = process.pid
                logger.debug(f"Routing handler PID: {routing_handler_pid}")
                
                # 等待一段时间，确保目的地已设置
                time.sleep(5)

                # if mode == 'replay':
                #     # 在回放模式下，等待更长时间确保路由被完全处理
                #     time.sleep(5)
                #     self.exec("pkill -2 -f 'routing_handler.py'")  # 使用SIGINT而不是直接kill
                # else:
                #     #FIXME: need to fix 
                #     # 手动终止routing_handler.py进程
                #     subprocess.run(["kill", str(routing_handler_pid)])
                #     logger.info(f"Terminated routing handler with PID {routing_handler_pid}")
                
            else:
                # 在同步模式下，process 是一个 CompletedProcess 对象
                if process.returncode != 0:
                    logger.error(f"Failed to set destination: {process.stderr}")
                    logger.error(f"Command output: {process.stdout}")
                    return None
                else:
                    logger.info("Successfully set destination")
                    print("Successfully set destination")
            
            # 如果需要，可以等待一段时间，确保目的地已设置
            
            return True
                
        except Exception as e:
            logger.error(f"Failed to set destination: {e}")
            print(f"Failed to set destination: {e}")
            raise
        
        
        
    #wei: ori version, will copy all logs to the output dir  
    # def clean_apollo_data(self, test_output_dir: str = None, is_replay: bool = False, segment: str = None):
    #     print("********************========================************")
    #     print("********************CLEAN APOLLO DATA!! CHECKOUT!!!************")
    #     print("********************========================************")
    #     print("test_output_dir is:", test_output_dir)
    #     """清理Apollo数据并保存日志"""
    #     try:
    #         if test_output_dir:
    #             container_logs_dir = self.convert_host_path_to_container(test_output_dir)
    #             print("container_log_dir is:", container_logs_dir)
                
    #             if is_replay and segment:
    #                 runtime_logs_dir = Path(container_logs_dir) / "replay_scenarios" / segment / "apollo_logs" / "runtime_logs"
    #             else:
    #                 runtime_logs_dir = Path(container_logs_dir) / "original_scenario" / "apollo_logs" / "runtime_logs"
    #             print("--- before make directory")
    #             # runtime_logs_dir.mkdir(parents=True, exist_ok=True)
                
    #             # 在容器内创建目录
    #             self.exec(f"mkdir -p '{container_logs_dir}'")
    #             # print("--- directory created in container")
                
    #             print("--- after make directory")
                
    #             # chmod命令
    #             chmod_cmd = f"chmod -R 777 {runtime_logs_dir}"
    #             self.exec(chmod_cmd)
    #             print("--- chmod -r directory")

                
    #             # 复制日志命令
    #             copy_cmd = """bash -c 'if [ -d "/apollo/data/log" ]; then \
    #                 cd /apollo/data/log && \
    #                 sudo tar czf - . | (cd {} && sudo tar xzf -) && \
    #                 sudo chown -R $(id -u):$(id -g) {}; \
    #             fi'""".format(runtime_logs_dir, runtime_logs_dir)
                
                
    #             logger.info(f"Saved Apollo runtime logs to {runtime_logs_dir}")
    #             print("--- before copy files to the target directory")
                
    #             self.exec(copy_cmd, verbose=True)
    #             print("--- after copy files to the target directory")
                
    #             logger.info(f"Saved Apollo runtime logs to {runtime_logs_dir}")
    #             print("================copy_cmd is:",copy_cmd)
    #         print("--- before clean the bag directory=")

    #         # 清理数据命令
    #         clean_cmd1 = """bash -c 'rm -rf /apollo/data/bag/*'"""
    #         self.exec(clean_cmd1, verbose=True)  # 添加verbose=True以查看输出
    #         clean_cmd2 = """bash -c 'rm -rf /apollo/data/core/*'"""
    #         self.exec(clean_cmd2, verbose=True)  # 添加verbose=True以查看输出
            
            
    #         logger.info("Cleaned Apollo data directories")
    #     except Exception as e:
    #         logger.error(f"Failed to clean Apollo data: {e}")
            

    #wei: new version, we only need to copy the latest log log files
    def clean_apollo_data(self, test_output_dir: str = None, is_replay: bool = False, segment: str = None):
        """
        清理Apollo数据并保存最新的日志。
        """
        print("********************========================************")
        print("********************CLEAN APOLLO DATA!! CHECKOUT!!!************")
        print("********************========================************")
        print("test_output_dir is:", test_output_dir)
        
        try:
            if test_output_dir:
                # 将主机路径转换为容器内路径
                container_logs_dir = self.convert_host_path_to_container(test_output_dir)
                print("container_log_dir is:", container_logs_dir)
                
                # 根据是否是回放模式来设置日志的目标目录
                if is_replay and segment:
                    runtime_logs_dir = Path(container_logs_dir) / "replay_scenarios" / segment / "apollo_logs" / "runtime_logs"
                else:
                    runtime_logs_dir = Path(container_logs_dir) / "original_scenario" / "apollo_logs" / "runtime_logs"

                # 创建目标目录
                self.exec(f"mkdir -p '{runtime_logs_dir}'")
                print("--- after making directory")

                # 修改目标目录权限
                chmod_cmd = f"chmod -R 777 {runtime_logs_dir}"
                self.exec(chmod_cmd)
                print("--- chmod -R directory")

                # 构造单行的 `copy_cmd`，移除换行符
                copy_cmd = (
                    f"bash -c 'if [ -d \"/apollo/data/log\" ]; then "
                    f"cd /apollo/data/log && "
                    f"for link in *.INFO; do "
                    f"if [ -L \"$link\" ]; then "
                    f"target=$(readlink -f \"$link\"); "
                    f"cp \"$target\" {runtime_logs_dir}/; "
                    f"fi; "
                    f"done && "
                    f"sudo chown -R $(id -u):$(id -g) {runtime_logs_dir}; "
                    f"fi'"
                )

                print("--- before copying files to the target directory")
                print(f"Copy command: {copy_cmd}")

                # 执行日志文件复制命令
                self.exec(copy_cmd, verbose=True)
                print("--- after copying files to the target directory")

                # 清理bag目录
                clean_cmd1 = """bash -c 'rm -rf /apollo/data/bag/*'"""
                self.exec(clean_cmd1, verbose=True)
                
                # 清理core目录
                clean_cmd2 = """bash -c 'rm -rf /apollo/data/core/*'"""
                self.exec(clean_cmd2, verbose=True)
                
                logger.info("Cleaned Apollo data directories")
        except Exception as e:
            logger.error(f"Failed to clean Apollo data: {e}")


# the terminal version of the above command, it is fine .
# docker exec -it apollo_dev_w bash -c '
# if [ -d "/apollo/data/log" ]; then
#     cd /apollo/data/log && \
#     for link in *.INFO; do  # 遍历所有 *.INFO 链接文件
#         if [ -L "$link" ]; then  # 检查文件是否是符号链接
#             target=$(readlink -f "$link")  # 获取链接指向的实际文件路径
#             cp "$target" /apollo/modules/carla_bridge/multi_vehicle_fuzz/test_outputs/20241204_145321/test_20241204_145544_v100_w70/original_scenario/apollo_logs/runtime_logs
#   # 复制最新日志文件到目标目录（替换为实际路径）
#         fi
#     done && \
#     sudo chown -R $(id -u):$(id -g) /apollo/modules/carla_bridge/multi_vehicle_fuzz/test_outputs/20241204_145321/test_20241204_145544_v100_w70/original_scenario/apollo_logs/runtime_logs
#  # 修改目标目录的文件所有权
# fi
# '




    def terminate_all_processes(self):
        """终止所有相关的进程"""
        try:
            # 停止所有Python脚本进程
            self.exec("pkill -f 'python.*main.py'")
            self.exec("pkill -f 'python.*record_data.py'")
            self.exec("pkill -f 'python.*generate_traffic.py'")
            self.exec("pkill -f 'python.*routing_handler.py'")
            
            # 停止cyber_recorder进程
            self.exec("pkill -f 'cyber_recorder'")
            
            # 等待进程终止
            time.sleep(2)
            
            # 检查是否还有残留进程
            check_cmd = "ps aux | grep -E 'main.py|record_data.py|generate_traffic.py|routing_handler.py|cyber_recorder' | grep -v grep"
            result = self.exec(check_cmd)
            
            if result.stdout.strip():
                logger.warning("Some processes are still running, force killing...")
                # 强制终止
                self.exec("pkill -9 -f 'python.*main.py'")
                self.exec("pkill -9 -f 'python.*record_data.py'")
                self.exec("pkill -9 -f 'python.*generate_traffic.py'")
                self.exec("pkill -9 -f 'python.*routing_handler.py'")
                self.exec("pkill -9 -f 'cyber_recorder'")
            
            logger.info("All processes terminated")
            
        except Exception as e:
            logger.error(f"Error terminating processes: {e}")
            raise
        
    
    def check_dreamview_status(self):
        print("********************Checking DreamView Status************")
        """在宿主机上检查 Dreamview 是否在运行"""
        try:
            # 直接在宿主机上执行 ps 命令
            cmd = "ps aux | grep 'dreamview' | grep -v grep"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

            if result.stdout.strip():
                # 如果有输出（找到了进程）
                logger.info("Dreamview is running")
                return True
            logger.warning("Dreamview is not running")
            return False
        except Exception as e:
            logger.error(f"Failed to check Dreamview status: {e}")
            return False

    def check_carla_status(self):
        print("********************Checking Carla Status************")
        """在宿主机上检查 Carla 是否在运行"""
        try:
            # 直接在宿主机上执行 ps 命令
            cmd = "ps aux | grep 'CarlaUE4' | grep -v grep"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

            if result.stdout.strip():
                # 如果有输出（找到了进程）
                logger.info("Carla is running")
                return True
            logger.warning("Carla is not running")
            return False
        except Exception as e:
            logger.error(f"Failed to check Carla status: {e}")
            return False
        
            
    def check_apollo_status(self):
        """检查 Apollo 核心模块是否在运行，并启动未运行的模块"""

        print("******************************************************")
        print("********************Checking Apollo Status************")
        print("******************************************************")

        try:
            # 定义需要检查的模块及其路径
            #TODO:why no perception  a
            modules = {
                "planning": "/apollo/modules/planning/dag/",
                "control": "/apollo/modules/control/dag/",
                "prediction": "/apollo/modules/prediction/dag/",
                
                "routing": "/apollo/modules/routing/dag/",
                "localization":"/apollo/modules/localization/dag/"
                # "guardian":"/apollo/modules/guardian/dag/"
                
                
                
            }

            # 初始化模块状态
            module_status = {module: False for module in modules}

            # 检查所有模块的运行状态
            cmd = "ps aux | grep mainboard | grep -v grep"
            result = self.exec(cmd)

            if result.returncode != 0:
                print("Failed to execute command or no processes found.")
                return False

            # 解析命令输出
            output = result.stdout.strip()
            print(f"Command Output:\n{output}")

            # 遍历模块，检查是否已经在运行
            for line in output.splitlines():
                for module, path in modules.items():
                    if path in line:  # 检查模块对应的路径是否出现在 ps aux 输出中
                        module_status[module] = True

            # 检查模块状态，并启动未运行的模块
            for module, is_running in module_status.items():
                if is_running:
                    print(f"{module}: Running")
                else:
                    print(f"{module}: Not Running. Starting {module}...")
                    # 调用对应模块的启动函数
                    start_function = getattr(self, f"start_{module}", None)
                    if callable(start_function):
                        start_function()
                    else:
                        print(f"Error: No start function defined for {module}.")

            return True

        except Exception as e:
            print(f"Error occurred while checking Apollo status: {e}")
            return False

    # def ensure_log_dirs(self):
    #     """确保必要的日志目录存在"""
    #     try:
    #         cmd = """
    #             mkdir -p /apollo/data/log
    #             mkdir -p /apollo/data/bag
    #             mkdir -p /apollo/data/core
    #             chmod -R 777 /apollo/data
    #         """
    #         self.exec(cmd)
    #         logger.info("Log directories created/verified")
    #     except Exception as e:
    #         logger.error(f"Failed to create log directories: {e}")
    #         raise
    
  
    def is_process_running(self, process_pattern: str) -> bool:
        """
        检查容器内是否有指定模式的进程在运行
        :param process_pattern: 要检查的进程模式
        :return: 如果进程在运行，返回 True；否则，返回 False
        """
        cmd = f'''ps aux | grep "{process_pattern}" | grep -v grep | wc -l'''
        
        try:
            result = self.exec(f"bash -c '{cmd}'", detached=False)
            if isinstance(result, subprocess.CompletedProcess):
                count = int(result.stdout.strip())
                logger.debug(f"Process count for '{process_pattern}': {count}")
                # 由于record进程会有两个实例
                return count >= 2 if "cyber_recorder" in process_pattern else count > 0
        except Exception as e:
            logger.error(f"Error checking process status: {e}")
            return False
        
        return False

    def start_apollo_recorder(self, test_output_dir: str, is_replay: bool = False, segment: str = None):
        """启动Apollo cyber_recorder记录所有话题"""
        try:
            # 构建容器内的日志存储路径
            container_logs_dir = self.convert_host_path_to_container(test_output_dir)
            if is_replay and segment:
                record_path = f"{container_logs_dir}/replay_scenarios/{segment}/apollo_logs/apollo.record"
            else:
                record_path = f"{container_logs_dir}/original_scenario/apollo_logs/apollo.record"
                
            # 确保目录存在
            os.path.dirname(record_path)
            self.exec(f"mkdir -p '{os.path.dirname(record_path)}'")
            
            # 启动录制命令
            cmd = f"""bash -c 'source /apollo/cyber/setup.bash && \
                cyber_recorder record -a -o "{record_path}" > /dev/null 2>&1'"""
                
            self.exec(cmd, detached=True)
            logger.info(f"Started Apollo recorder, saving to {record_path}")
            
            # 验证录制进程是否启动
            if not self._verify_apollo_recorder():
                raise RuntimeError("Failed to start Apollo recorder")
                
            return True
            
        except Exception as e:
            logger.error(f"Failed to start Apollo recorder: {e}")
            return False

    def _verify_apollo_recorder(self, max_retries: int = 10, retry_interval: int = 1) -> bool:
        """验证Apollo recorder是否正在运行"""
        for i in range(max_retries):
            if self.is_process_running("cyber_recorder.*record"):
                logger.info("Apollo recorder is running")
                return True
            time.sleep(retry_interval)
            logger.debug(f"Waiting for Apollo recorder to start (attempt {i+1}/{max_retries})")
        logger.error("Failed to verify Apollo recorder is running")
        return False

    def stop_apollo_recorder(self):
        print("==========================================================")
        print("========== stop apollo recorder in container  =======")
        print("==========================================================")
        
        """停止Apollo cyber_recorder进程"""
        try:
            logger.info("Stopping Apollo recorder...")
            # 使用SIGINT信号停止录制进程
            self.exec("pkill --signal SIGINT -f 'cyber_recorder record'")
            time.sleep(2)  # 给进程一些时间来保存数据
            
            # 检查进程是否已经停止
            if self.is_process_running("cyber_recorder.*record"):
                logger.warning("Apollo recorder still running, force killing...")
                self.exec("pkill -9 -f 'cyber_recorder record'")
            
            logger.info("Apollo recorder stopped")
            return True
            
        except Exception as e:
            logger.error(f"Failed to stop Apollo recorder: {e}")
            return False
            
    # def is_process_running(self, process_name: str) -> bool:
    #     """
    #     检查容器内是否有指定名称的进程在运行
    #     :param process_name: 要检查的进程名称
    #     :return: 如果进程在运行，返回 True；否则，返回 False
    #     """
    #     cmd = f"pgrep -f {process_name}"
    #     result = self.exec(f"bash -c '{cmd}'", detached=False)
    #     if isinstance(result, subprocess.CompletedProcess):
    #         return result.returncode == 0
    #     return False
        
        
        
    def convert_host_path_to_container(self, host_path: str) -> str:
        """将主机路径转换为容器内路径"""
        # 假设PROJECT_ROOT映射到容器内的/apollo/multi_vehicle_fuzz
        try:
            rel_path = os.path.relpath(host_path, PROJECT_ROOT)
            container_path = os.path.join("/apollo/modules/carla_bridge/multi_vehicle_fuzz", rel_path)
            return container_path
        except ValueError:
            # 如果路径不在PROJECT_ROOT下,返回原路径并记录警告
            logger.warning(f"Path {host_path} is not under PROJECT_ROOT, using as is")
            return host_path