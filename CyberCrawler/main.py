"""
CyberCrawler entry point
Initialize robot + remote control, enter main loop

I2C pin assignment:
  PCA9685: SCL=Pin(2), SDA=Pin(3), hardware I2C(0)
  MPU6050: SCL=Pin(1), SDA=Pin(0), SoftI2C (software emulation, any pins)

Controls:
  L1 3-position switch → VEHICLE / IDLE / CRAWL
  K1 short press → light toggle
"""
from machine import Pin, I2C, SoftI2C
from robot import Robot
from remote import RemoteControl
from calibration import I2C_PCA, I2C_IMU
import time


def _create_i2c(cfg):
    if cfg.get('type') == 'soft':
        return SoftI2C(scl=Pin(cfg['scl']), sda=Pin(cfg['sda']))
    else:
        return I2C(cfg['bus'], scl=Pin(cfg['scl']), sda=Pin(cfg['sda']))


def main():
    print("[MAIN] CyberCrawler starting...")

    i2c_pca = _create_i2c(I2C_PCA)
    print("[MAIN] I2C PCA type=" + I2C_PCA['type'] +
          " SCL=" + str(I2C_PCA['scl']) +
          " SDA=" + str(I2C_PCA['sda']))

    i2c_imu = None
    try:
        i2c_imu = _create_i2c(I2C_IMU)
        print("[MAIN] I2C IMU type=" + I2C_IMU['type'] +
              " SCL=" + str(I2C_IMU['scl']) +
              " SDA=" + str(I2C_IMU['sda']))
    except Exception as e:
        print(f"[MAIN] I2C IMU bus init failed: {e}")

    robot = Robot(i2c_pca=i2c_pca, i2c_imu=i2c_imu)
    remote = RemoteControl()

    robot.home()
    print("[MAIN] Robot homed.")
    time.sleep(1)
    print("[MAIN] Starting main loop.")

    loop_count = 0
    fps_timer = time.ticks_us()
    last = time.ticks_us()

    while True:
        now = time.ticks_us()
        dt = time.ticks_diff(now, last) / 1_000_000.0
        dt = max(0.001, min(0.1, dt))
        last = now

        commands = remote.get_commands()
        robot.update(dt, commands)

        # FPS statistics
        loop_count += 1
        if loop_count >= 100:
            elapsed = time.ticks_diff(time.ticks_us(), fps_timer) / 1_000_000.0
            print("[MAIN] " + str(int(100 / elapsed)) + " Hz")
            loop_count = 0
            fps_timer = time.ticks_us()

        # 100Hz speed limit
        elapsed = time.ticks_diff(time.ticks_us(), now)
        if elapsed < 10000:
            time.sleep_ms(max(1, (10000 - elapsed) // 1000))


main()
