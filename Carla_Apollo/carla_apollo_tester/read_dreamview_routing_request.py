import pyshark

# 捕获本地回环接口上的流量
capture = pyshark.LiveCapture(interface='lo', display_filter='tcp.port == 8888')

print("Listening for Dreamview RoutingRequest packets on port 8888...")
for packet in capture.sniff_continuously():
    try:
        if hasattr(packet.tcp, 'payload'):
            # 从包中提取载荷
            payload = bytes.fromhex(packet.tcp.payload.replace(':', '')).decode('utf-8', errors='ignore')
            
            # 检查是否包含目标关键字
            if "RoutingRequest" in payload:
                print("Captured RoutingRequest packet:")
                print(payload)

                # 如果是 JSON，解析并打印
                try:
                    data = json.loads(payload)
                    print("Parsed JSON:")
                    print(json.dumps(data, indent=4))
                except json.JSONDecodeError:
                    print("Payload is not valid JSON.")
    except Exception as e:
        print(f"Error processing packet: {e}")