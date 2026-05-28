import smbus2
import glob

print("=" * 60)
print("             CarrotPilot I2C Bus & Device Scanner")
print("=" * 60)

found_any = False
for bus_path in sorted(glob.glob("/dev/i2c-*")):
    try:
        bus_num = int(bus_path.rsplit("-", 1)[1])
        print(f"\n🔍 Scanning Bus {bus_num} ({bus_path})...")
        
        # 尝试打开总线
        with smbus2.SMBus(bus_num) as bus:
            found = []
            for addr in range(0x03, 0x78):
                try:
                    # 读取寄存器 0x0F (WHO_AM_I)
                    chip_id = bus.read_byte_data(addr, 0x0F)
                    found.append(f"0x{addr:02X} (WHO_AM_I=0x{chip_id:02X})")
                except OSError:
                    # 如果读寄存器不成功，尝试直接 ping 地址
                    try:
                        bus.read_byte(addr)
                        found.append(f"0x{addr:02X} (Responsive)")
                    except OSError:
                        continue
            
            if found:
                print(f"  🎉 >>> FOUND DEVICES: {', '.join(found)}")
                found_any = True
            else:
                print("  (No responsive devices on this bus)")
    except OSError as e:
        print(f"  ❌ Cannot open bus {bus_path}: {e}")

print("\n" + "=" * 60)
if not found_any:
    print("⚠️  Warning: No active I2C devices were found on any bus.")
    print("Please check: 1. USB adapter wiring, 2. Power supply, 3. SDA/SCL pull-up resistors.")
else:
    print("Please use the detected Bus and Address to update your configuration.")
print("=" * 60)
