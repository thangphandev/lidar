import math
import pygame
import time
from collections import deque
from pyrplidar import PyRPlidar

# --- CẤU HÌNH ---
WIDTH, HEIGHT = 700, 700
CENTER = (WIDTH // 2, HEIGHT // 2)
MIN_DISTANCE = 150    # mm (Loại bỏ nhiễu quá gần)
MAX_DISTANCE = 2500   # mm (Loại bỏ giá trị quá lớn/nhiễu xa)
SCALE = (WIDTH // 2) / MAX_DISTANCE

# Cấu hình lọc nhiễu
SMOOTHING_WINDOW = 3  # Số mẫu dùng để tính trung bình (càng cao càng mượt nhưng càng chậm)

# Màu sắc
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
DARK_GREEN = (0, 60, 0)

def run_radar_filtered():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Lidar Filtered Radar - Pi 3B+")
    clock = pygame.time.Clock()

    # Tạo background tĩnh
    background = pygame.Surface((WIDTH, HEIGHT))
    background.fill(BLACK)
    for r in range(500, MAX_DISTANCE + 1, 500):
        pygame.draw.circle(background, DARK_GREEN, CENTER, int(r * SCALE), 1)

    # Bộ nhớ đệm cho lọc nhiễu: {góc: deque([d1, d2, d3])}
    filter_buffer = {ang: deque(maxlen=SMOOTHING_WINDOW) for ang in range(360)}
    final_map = {}

    lidar = PyRPlidar()
    try:
        lidar.connect(port="/dev/ttyUSB0", baudrate=115200, timeout=3)
        lidar.set_motor_pwm(400)
        time.sleep(1)
        scan_generator = lidar.start_scan()
    except Exception as e:
        print(f"Lỗi kết nối: {e}")
        return

    last_draw_time = time.time()

    try:
        for scan in scan_generator():
            for event in pygame.event.get():
                if event.type == pygame.QUIT: return

            # 1. LỌC NGƯỠNG (Loại bỏ giá trị quá lớn hoặc quá nhỏ)
            dist = scan.distance
            if MIN_DISTANCE <= dist <= MAX_DISTANCE:
                angle = int(scan.angle) % 360
                
                # 2. LỌC TRUNG BÌNH (Moving Average)
                filter_buffer[angle].append(dist)
                
                # Tính giá trị trung bình từ các mẫu trong buffer
                avg_dist = sum(filter_buffer[angle]) / len(filter_buffer[angle])
                final_map[angle] = avg_dist
            
            # Tối ưu việc vẽ để Pi 3B+ không bị treo
            current_time = time.time()
            if current_time - last_draw_time > 0.1: # 10 FPS
                screen.blit(background, (0, 0))
                
                for ang, dist in list(final_map.items()):
                    # Chuyển tọa độ
                    rad = math.radians(ang - 90)
                    r = dist * SCALE
                    px = CENTER[0] + int(r * math.cos(rad))
                    py = CENTER[1] + int(r * math.sin(rad))
                    
                    # Vẽ điểm: Càng gần vật cản càng đỏ để cảnh báo
                    color = RED if dist < 600 else GREEN
                    pygame.draw.circle(screen, color, (px, py), 3)

                pygame.display.flip()
                last_draw_time = current_time

    except KeyboardInterrupt:
        print("Đang dừng...")
    finally:
        lidar.stop()
        lidar.set_motor_pwm(0)
        lidar.disconnect()
        pygame.quit()

if __name__ == "__main__":
    run_radar_filtered()
