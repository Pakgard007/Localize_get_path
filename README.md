# 2Localize_get_path — Localization และขับรถกอล์ฟตามเส้นทาง

Workspace ROS 2 (Humble) สำหรับ **หาตำแหน่งรถกอล์ฟบนแผนที่ 3 มิติ** (NDT localization ด้วย LiDAR Ouster)
แล้ว **ขับรถอัตโนมัติตามเส้นทาง** (Pure Pursuit + CBF-QP)

แผนที่ `.pcd` ที่ใช้ทำมาจาก workspace **1Mapping** (LIO-SAM)

```text
Ouster OS-1-128 ──/ouster/points──► lidar_localization_ros2 (NDT) ──/pcl_pose──┐
GlobalMap.pcd ───────────────────►                                             │
u-blox ZED-F9R + NTRIP ──/fix──► (ตั้งตำแหน่งเริ่มต้น)                            ▼
                                       nav_path.py (/nav_path, v_des) ──► pagard_adaptive_fixed.py
                                                                          (Pure Pursuit + Safety)
                                                                             ▲         │
                                                         cbf_qp_node (v*, L_d*)        │ /cmd_vel
                                                                                       ▼
                                          golfcart_low_level ──CAN──► STM32 (คันเร่ง/เบรก) + KEYA (พวงมาลัย)
```

## โครงสร้างโปรเจกต์

| โฟลเดอร์ / ไฟล์ | หน้าที่ |
|---|---|
| `src/lidar_localization_ros2-main/` | NDT localization เทียบสแกนกับแผนที่ → `/pcl_pose` พร้อมโมเดลรถใน RViz — ดู [GOLFCART_SETUP.md](src/lidar_localization_ros2-main/GOLFCART_SETUP.md) |
| `src/ndt_omp_ros2-humble/` | NDT/GICP แบบ OpenMP (ใช้โดย localization) |
| `src/cart_rviz_panel/` | หน้าต่าง RViz มีปุ่มเปิด/ปิดแต่ละขั้น: LiDAR, Localization, Low-level, GPS, NTRIP, CBF-QP, Logger |
| `src/sim/` | สคริปต์ขับรถ: `nav_path.py` (ส่งเส้นทาง), `pagard_adaptive_fixed.py` (waypoint follower), `adaptive_ld_node_fixed.py`, ไฟล์เส้นทาง `loop*_qp_comfort.csv`, สคริปต์ทดสอบ/บันทึก |
| `src/nav_path_launch/` | launch ของ `nav_path.py` และสายควบคุม CBF-QP (`cbf_qp_node`, logger CSV) |
| `src/campus_buildings/` | วาดตึกในมหาวิทยาลัยบนแผนที่ใน RViz (`/campus_buildings`) และตั้ง initial pose อัตโนมัติ |
| `src/golfcart_low_level/` | สะพาน ROS 2 ↔ CAN ไปบอร์ด STM32 และไดรเวอร์พวงมาลัย KEYA — ดู [README](src/golfcart_low_level/README.md) |
| `src/ouster-ros/`, `src/ublox/`, `src/ntrip_client/` | ไดรเวอร์ LiDAR, GNSS และ NTRIP client |
| `src/old_maps/` | แผนที่เก่า (เลิกใช้แล้ว) — ดู [README](src/old_maps/README.md) |
| `firmware/stm32_motorbrake_car_shield/` | เฟิร์มแวร์ STM32G474 ควบคุมความเร็วและเบรก |
| `scripts/` | `pause_driving.sh` / `resume_driving.sh` (หยุด/ขับต่อทันที), `start_low_level.sh` / `stop_low_level.sh`, `calibrate_odom.py`, `add_pose_stamp.py` |
| `start_cart.sh` | รีเซ็ต CAN, รอ STM32 ตอบ แล้วเปิด `golfcart_low_level` |
| `can_check.sh` | ตรวจสุขภาพ CAN bus ครั้งเดียว (รันตอนปิด ROS อยู่) |
| `docs/` | ตัวอย่างการสั่งเบรก, ภาพรวมระบบ low-level |

## ⚠️ ไฟล์แผนที่ไม่ได้อยู่ใน git

ไฟล์ `.pcd` ใหญ่เกินที่ GitHub รับ (จำกัดไฟล์ละ 100MB) จึงไม่ได้อัพขึ้นมา:

| ไฟล์ | ขนาด |
|---|---|
| `src/FullGlobalMap.pcd` | 554 MB (แผนที่ที่ใช้อยู่) |
| `src/old_maps/GlobalMapLoop1.pcd` | 90 MB |
| `src/old_maps/GlobalMapLoop2.pcd` | 223 MB |

หลัง clone ต้องคัดลอก `FullGlobalMap.pcd` มาวางที่ `src/` เอง (หรือทำแผนที่ใหม่ด้วย 1Mapping)
แล้วตรวจ `map_path` ใน `src/lidar_localization_ros2-main/param/localization.yaml`

## ติดตั้งและ build

```bash
git clone <url ของ repo นี้> ~/2Localize_get_path
cd ~/2Localize_get_path
# วาง FullGlobalMap.pcd ไว้ที่ src/
rosdep install --from-paths src --ignore-src -r -y
sudo apt install -y python3-can can-utils python3-aenum
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### บัญชี NTRIP

username/password ของ NTRIP **ไม่ได้อยู่ใน git** ให้สร้าง `src/ntrip_client/config/ntrip.local.yaml`
(อยู่ใน `.gitignore`) แล้ว build ใหม่ — launch จะโหลดไฟล์นี้ทับ `ntrip.yaml`

```yaml
ntrip_client:
  ros__parameters:
    username: "<username>"
    password: "<password>"
```

## วิธีใช้งาน

### แบบใช้ Cart Panel (แนะนำ)

```bash
source /opt/ros/humble/setup.bash
source ~/2Localize_get_path/install/setup.bash
ros2 launch cart_rviz_panel cart_panel.launch.py      # light:=true = ไม่โหลดแผนที่ (เครื่องช้า)
```

กดเปิดทีละขั้นในแผง:

1. **LiDAR Ouster** — `ros2 launch ouster_ros driver.launch.py`
2. **Localization** — `ros2 launch lidar_localization_ros2 lidar_localization.launch.py`
3. **Low-level** — `bash ~/2Localize_get_path/start_cart.sh` (CAN + STM32 + พวงมาลัย)
4. **GPS u-blox** — `ros2 launch ublox_gps ublox_gps_node_zed_f9r-launch.py`
5. **NTRIP (RTK)** — `ros2 launch ntrip_client ntrip_client-launch.py`
6. **CBF-QP** — `ros2 launch nav_path_launch cbf_qp.launch.py tune:=true` (สายควบคุมการขับ)
7. **Logger CSV** — บันทึกค่าควบคุมเป็น CSV

### หยุดรถฉุกเฉินจากซอฟต์แวร์

```bash
bash ~/2Localize_get_path/scripts/pause_driving.sh    # ส่งความเร็ว 0 ค้างไว้ทันที
bash ~/2Localize_get_path/scripts/resume_driving.sh   # ขับต่อ
```

> ⚠️ ตอนรถวิ่งอัตโนมัติ ต้องมีคนนั่งพร้อมกดปุ่มหยุดฉุกเฉิน (e-stop) ของรถเสมอ สคริปต์นี้ไม่ได้ใช้แทนปุ่มนั้น

## ใช้คู่กับ 1Mapping

ทั้งสอง workspace มี `golfcart_low_level`, `ouster_ros`, `ublox_gps` ซ้ำกัน และ ROS 2 ใช้ของ
workspace ที่ **source ทีหลัง** — ตอนขับรถให้ source `2Localize_get_path` ทีหลังเสมอ
(`ublox_gps_node_zed_f9r-launch.py` มีเฉพาะใน workspace นี้)

## แก้ปัญหาเบื้องต้น

- **CAN ค้าง / `Transmit buffer full`** — ปิด ROS แล้วรัน `bash can_check.sh` (ระหว่างนั้นปิด-เปิดบอร์ด STM32)
- **`/pcl_pose` ไม่ออก** — ตรวจว่า `/ouster/points` มีข้อมูล และ `map_path` ชี้ไปที่ไฟล์ `.pcd` ที่มีอยู่จริง
- **ไม่เจอ GPS** — `ls /dev/ttyACM*` และ `dmesg | tail -20`
- **ไม่มี `/ouster/points`** — `ping 192.168.1.100`

## เครดิต

- [lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2), [ndt_omp](https://github.com/koide3/ndt_omp) — rsasaki0109, koide3
- [ouster-ros](https://github.com/ouster-lidar/ouster-ros) — Ouster
- [ublox](https://github.com/KumarRobotics/ublox) — KumarRobotics

ไลเซนส์ของแต่ละแพ็กเกจเป็นไปตามไฟล์ `LICENSE` ในโฟลเดอร์นั้น
