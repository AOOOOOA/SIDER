from pathlib import Path
from typing import Union
from config import PROJECT_ROOT, PATH_MAPPINGS

class PathManager:
    def __init__(self):
        self.project_root = Path(PROJECT_ROOT)
        self.container_root = Path(PATH_MAPPINGS["PROJECT_ROOT"])
        
    def to_container_path(self, host_path: Union[str, Path]) -> str:
        """转换为容器内路径"""
        host_path = Path(host_path)
        try:
            rel_path = host_path.relative_to(self.project_root)
            return str(self.container_root / rel_path)
        except ValueError:
            raise ValueError(f"Path {host_path} must be under {self.project_root}")
            
    def to_host_path(self, container_path: Union[str, Path]) -> str:
        """转换为主机路径"""
        container_path = Path(container_path)
        try:
            rel_path = container_path.relative_to(self.container_root)
            return str(self.project_root / rel_path)
        except ValueError:
            raise ValueError(f"Path {container_path} must be under {self.container_root}")
            
    def validate_path(self, path: Union[str, Path]) -> bool:
        """验证路径"""
        try:
            Path(path).relative_to(self.project_root)
            return True
        except ValueError:
            return False

# 创建全局实例
path_manager = PathManager()