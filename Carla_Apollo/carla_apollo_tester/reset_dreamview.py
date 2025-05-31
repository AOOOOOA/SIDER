#!/usr/bin/env python3

from websocket import create_connection
import json
import time
import sys
import os
import logging
from loguru import logger

class DreamviewConnection:
    def __init__(self, ip=os.environ.get("DREAMVIEW_HOST", "localhost"), port="8888"):
        """初始化Dreamview连接"""
        self.url = f"ws://{ip}:{port}/websocket"
        try:
            self.ws = create_connection(self.url)
            logger.info(f"Successfully connected to Dreamview at {self.url}")
        except Exception as e:
            logger.error(f"Failed to connect to Dreamview: {str(e)}")
            raise

    def reset_backend(self):
        """重置后端数据"""
        try:
            # 发送reset消息，格式与Dreamview一致
            message = {
                "type": "Reset"
            }
            self.ws.send(json.dumps(message))
            logger.info("Sent reset command")
            
            # 等待确保重置完成
            time.sleep(1)
            return True
            
        except Exception as e:
            logger.error(f"Failed to reset backend data: {str(e)}")
            return False
        
        
    def reset_all(self):
        try:
            message={
                "type":"HMIAction",
                "action":"RESET_MODE"
            }
            self.ws.send(json.dumps(message))
            logger.info("Sent Reset All command")
            
            time.sleep(1)
            return True
            
        except Exception as e:
            logger.error(f"Failed to reset all in dreamview: {str(e)}")
            return False
        
    def setup(self):
        
        try:
            message={
                "type":"HMIAction",
                "action":"SETUP_MODE"
            }
            self.ws.send(json.dumps(message))
            logger.info("Sent setup command")
            
            time.sleep(1)
            return True
            
        except Exception as e:
            logger.error(f"Failed to run setup in dreamview: {str(e)}")
            return False
        
        

    def reconnect(self):
        """重新连接websocket"""
        try:
            self.ws.close()
            self.ws = create_connection(self.url)
            logger.info("WebSocket connection reconnected")
        except Exception as e:
            logger.error(f"Error reconnecting: {str(e)}")
            raise

    def close(self):
        """关闭连接"""
        try:
            if hasattr(self, 'ws'):
                self.ws.close()
                logger.info("WebSocket connection closed")
        except Exception as e:
            logger.error(f"Error closing connection: {str(e)}")

def main():
    dream_conn = None
    try:
        # 连接Dreamview
        dream_conn = DreamviewConnection()

        # 重置backend数据
        logger.info("Attempting to reset backend data...")
        if dream_conn.reset_backend():
            logger.info("Backend data reset successful")
        else:
            logger.error("Failed to reset backend data")
            return 1

    except KeyboardInterrupt:
        logger.info("Operation cancelled by user")
        return 0
    except Exception as e:
        logger.error(f"An error occurred: {str(e)}")
        return 1
    finally:
        if dream_conn:
            dream_conn.close()

if __name__ == "__main__":
    # 配置日志
    logging.basicConfig(level=logging.INFO)
    sys.exit(main())