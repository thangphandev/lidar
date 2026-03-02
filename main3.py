import math
import pygame
import time
from collections import deque
from statistics import median
from pyrplidar import PyRPlidar

# --- CẤU HÌNH ---
WIDTH, HEIGHT = 700, 700
CENTER = (WIDTH // 2, HEIGHT // 2)
MIN_DISTANCE = 150      # mm - Loại bỏ nhiễu quá gần
MAX_DISTANCE = 2500     # mm - Loại bỏ giá trị quá xa
SCALE = (WIDTH // 2) / MAX_DISTANCE

# Lọc nhiễu
SMOOTHING_WINDOW = 5    # Số mẫu lưu để tính median (5 là tốt nhất cho A1M8)
POINT_LIFETIME = 0.8    # Giây - điểm sẽ bị xóa nếu không được cập nhật
DRAW_INTERVAL = 0.05    # Giây - 20 FPS (Pi 3B+ đủ sức)

# Ngưỡng lọc nhiễu nâng cao
MAX_JUMP = 400          # mm - Loại bỏ nếu khoảng cách nhảy quá lớn so với mẫu trước
MIN_QUALITY = 10        # Chất lượng tín hiệu tối thiểu (nếu pyrplidar hỗ trợ)

# Màu sắc
BLACK      = (0, 0, 0)
GREEN      = (0, 255, 0)
YELLOW     = (255, 255, 0)
RED        = (255, 60, 60)
DARK_GREEN = (0, 60, 0)
GRAY       = (40, 40, 40)
WHITE      = (220, 220, 220)
CYAN       = (0, 200, 255)

# Ngưỡng màu cảnh báo (mm)
ZONE_RED    = 500
ZONE_YELLOW = 1000


def build_background():
    """Vẽ nền tĩnh một lần để tiết kiệm CPU."""
    surf = pygame.Surface((WIDTH, HEIGHT))
    surf.fill(BLACK)

    # Vòng tròn khoảng cách
    for r_mm in range(500, MAX_DISTANCE + 1, 500):
        radius_px = int(r_mm * SCALE)
        pygame.draw.circle(surf, DARK_GREEN, CENTER, radius_px, 1)
        # Nhãn khoảng cách
        font = pygame.font.SysFont("monospace", 12)
        label = font.render(f"{r_mm // 1000:.1f}m", True, DARK_GREEN)
        surf.blit(label, (CENTER[0] + radius_px + 3, CENTER[1] - 8))

    # Đường trục
    pygame.draw.line(surf, GRAY, (CENTER[0], 0), (CENTER[0], HEIGHT), 1)
    pygame.draw.line(surf, GRAY, (0, CENTER[1]), (WIDTH, CENTER[1]), 1)

    # Điểm trung tâm (vị trí LIDAR)
    pygame.draw.circle(surf, CYAN, CENTER, 5)

    return surf


def get_point_color(dist_mm):
    """Trả về màu theo vùng cảnh báo."""
    if dist_mm < ZONE_RED:
        return RED
    elif dist_mm < ZONE_YELLOW:
        return YELLOW
    return GREEN


def run_radar_filtered():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("LIDAR A1M8 - Filtered Radar")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 14)

    background = build_background()

    # --- Bộ lọc & dữ liệu ---
    # filter_buffer[angle] = deque các mẫu gần nhất
    filter_buffer = {ang: deque(maxlen=SMOOTHING_WINDOW) for ang in range(360)}

    # final_map[angle] = (filtered_distance, timestamp)
    final_map = {}

    # Mẫu trước để lọc jump
    prev_raw = {}

    # --- Kết nối LIDAR ---
    lidar = PyRPlidar()
    try:
        lidar.connect(port="/dev/ttyUSB0", baudrate=115200, timeout=3)
        lidar.set_motor_pwm(400)
        time.sleep(1)
        scan_generator = lidar.start_scan()
    except Exception as e:
        print(f"[LỖI] Kết nối thất bại: {e}")
        pygame.quit()
        return

    print("[OK] LIDAR đã kết nối. Nhấn Ctrl+C hoặc đóng cửa sổ để thoát.")

    last_draw_time = time.time()
    point_count = 0

    try:
        for scan in scan_generator():
            # --- Xử lý sự kiện ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return

            dist = scan.distance
            angle_raw = scan.angle

            # --- LỌC 1: Ngưỡng khoảng cách ---
            if not (MIN_DISTANCE <= dist <= MAX_DISTANCE):
                continue

            angle = int(angle_raw) % 360

            # --- LỌC 2: Jump filter (loại bỏ nhảy đột ngột) ---
            if angle in prev_raw:
                if abs(dist - prev_raw[angle]) > MAX_JUMP:
                    # Bỏ qua điểm nhiễu nhảy lớn
                    prev_raw[angle] = dist
                    continue
            prev_raw[angle] = dist

            # --- LỌC 3: Median filter (chống nhiễu tốt hơn moving average) ---
            filter_buffer[angle].append(dist)
            if len(filter_buffer[angle]) >= 3:  # Cần ít nhất 3 mẫu
                filtered_dist = median(filter_buffer[angle])
            else:
                filtered_dist = dist

            # Cập nhật bản đồ với timestamp
            final_map[angle] = (filtered_dist, time.time())
            point_count += 1

            # --- VẼ theo interval ---
            now = time.time()
            if now - last_draw_time >= DRAW_INTERVAL:
                screen.blit(background, (0, 0))

                # Xóa điểm cũ (không còn được cập nhật)
                stale_angles = [
                    a for a, (_, ts) in final_map.items()
                    if now - ts > POINT_LIFETIME
                ]
                for a in stale_angles:
                    del final_map[a]
                    filter_buffer[a].clear()

                # Vẽ tất cả điểm hiện tại
                for ang, (dist_f, _) in list(final_map.items()):
                    rad = math.radians(ang - 90)
                    r_px = dist_f * SCALE
                    px = CENTER[0] + int(r_px * math.cos(rad))
                    py = CENTER[1] + int(r_px * math.sin(rad))

                    # Kiểm tra nằm trong màn hình
                    if 0 <= px < WIDTH and 0 <= py < HEIGHT:
                        color = get_point_color(dist_f)
                        pygame.draw.circle(screen, color, (px, py), 3)

                # --- HUD: Thông tin góc trái trên ---
                hud_lines = [
                    f"Diem: {len(final_map)}/360",
                    f"FPS : {clock.get_fps():.1f}",
                    f"Scan: {point_count}",
                ]
                for i, line in enumerate(hud_lines):
                    txt = font.render(line, True, WHITE)
                    screen.blit(txt, (8, 8 + i * 18))

                # --- Chú thích màu ---
                legend = [
                    (RED,    f"< {ZONE_RED}mm"),
                    (YELLOW, f"< {ZONE_YELLOW}mm"),
                    (GREEN,  f">= {ZONE_YELLOW}mm"),
                ]
                for i, (col, lbl) in enumerate(legend):
                    pygame.draw.circle(screen, col, (12, HEIGHT - 55 + i * 18), 5)
                    txt = font.render(lbl, True, WHITE)
                    screen.blit(txt, (22, HEIGHT - 62 + i * 18))

                pygame.display.flip()
                clock.tick()
                last_draw_time = now

    except KeyboardInterrupt:
        print("\n[INFO] Đang dừng...")
    finally:
        print("[INFO] Ngắt kết nối LIDAR...")
        lidar.stop()
        lidar.set_motor_pwm(0)
        lidar.disconnect()
        pygame.quit()
        print("[OK] Đã thoát.")


if __name__ == "__main__":
    run_radar_filtered()
