from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# รัน nav_path.py ตัวเดิมใน src/sim โดยตรง (ไม่ copy เข้า install) เพื่อให้
# - แก้ nav_path.py แล้วมีผลทันทีโดยไม่ต้อง colcon build ใหม่
# - SCRIPT_DIR ใน nav_path.py ยังชี้ src/sim จึงหา loop1_qp_comfort.csv เจอเหมือนเดิม
# ไม่ส่ง parameter ใด ๆ: ค่าทั้งหมดมาจาก declare_parameter ใน nav_path.py
# เหมือนตอนรัน /usr/bin/python3 .../src/sim/nav_path.py ด้วยมือ
DEFAULT_SCRIPT = '/home/myuser/2Localize_get_path/src/sim/nav_path.py'


def generate_launch_description():
    script = LaunchConfiguration('script')

    return LaunchDescription([
        DeclareLaunchArgument(
            'script',
            default_value=DEFAULT_SCRIPT,
            description='Path to nav_path.py (CsvPathPublisher).',
        ),
        Node(
            executable='/usr/bin/python3',
            arguments=[script],
            output='screen',
        ),
    ])
