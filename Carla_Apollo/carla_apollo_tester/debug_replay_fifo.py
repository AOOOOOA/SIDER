import socket

def setup_tcp_server(port):
    """设置TCP服务器"""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', port))
    server_socket.listen(1)
    return server_socket

def receive_tcp_data(server_socket):
    """接收TCP数据"""
    print("等待客户端连接...")
    conn, addr = server_socket.accept()
    print(f"客户端 {addr} 已连接")
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            print("接收到的数据:", data.decode())
    except Exception as e:
        print(f"接收数据时出错: {e}")
    finally:
        conn.close()
        print("客户端连接已关闭")

# 示例使用
if __name__ == "__main__":
    port = 12345
    server_socket = setup_tcp_server(port)
    print(f"TCP服务器正在监听端口 {port}")
    try:
        receive_tcp_data(server_socket)
    finally:
        server_socket.close()
        print("服务器已关闭")