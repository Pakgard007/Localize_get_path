import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# RViz ของรถ: display ทั้งหมดจาก localization.rviz ของ lidar_localization_ros2
# (อ่านใหม่ทุกครั้งที่เปิด แก้ที่ไฟล์นั้นไฟล์เดียวพอ) + panel ปุ่มเปิด/ปิด
# theme:=true (ค่าเริ่มต้น) = หน้าต่างธีมมืด, map สีเรียบ, สแกนสดสีสด, เส้นทางหนาขึ้น,
#   panel กับ Displays เป็นแท็บคู่กัน, ซ่อนเครื่องมือที่ไม่มีใครใช้
# theme:=false = หน้าตาเดิมของ localization.rviz (แค่เพิ่ม panel)
# light:=true = rviz ตัวเบาที่มีแค่ panel ไม่โหลด map 5.8M จุด
# ขั้นตอนที่เปิดจากปุ่มรันในหน้าต่าง terminal ของตัวเอง ปิด RViz นี้แล้วยังรันต่อ

PANEL = {'Class': 'cart_rviz_panel/CartPanel', 'Name': 'ควบคุมรถ'}

# The map is the backdrop: muted blue by intensity instead of a height rainbow,
# so the live scan, the paths and the cart stand out on it.
MAP_STYLE = {
    'Color Transformer': 'Intensity', 'Channel Name': 'intensity', 'Use rainbow': False,
    'Min Color': '30; 48; 72', 'Max Color': '140; 185; 225',
    'Autocompute Intensity Bounds': True, 'Style': 'Points', 'Size (Pixels)': 2, 'Alpha': 1.0,
}
SCENE = {
    'InitialMap': MAP_STYLE,
    'Map': MAP_STYLE,
    'LiveScan': {
        'Color Transformer': 'FlatColor', 'Color': '255; 186; 73',
        'Style': 'Flat Squares', 'Size (m)': 0.05, 'Alpha': 1.0,
    },
    # Same hues as before (route red, driven trail green), drawn as ribbons.
    'TrackedPath': {'Color': '248; 113; 113', 'Line Style': 'Billboards', 'Line Width': 0.2},
    'GlobalPath': {'Color': '74; 222; 128', 'Line Style': 'Billboards', 'Line Width': 0.12},
    'Grid': {'Color': '120; 130; 145', 'Alpha': 0.25, 'Cell Size': 10, 'Plane Cell Count': 30},
}
BACKGROUND = '18; 20; 24'
# The edges of the CBF-QP's h1 corridor (|e_lat| = e_max) either side of the
# route, from cbf_qp_node.py while it runs: blue while h1 is in use, grey when
# it is switched off or under Pure Pursuit. Drawn right after the route.
H1_CORRIDOR = {
    'Class': 'rviz_default_plugins/MarkerArray',
    'Enabled': True,
    'Name': 'H1Corridor',
    'Namespaces': {'h1_corridor': True},
    'Topic': {
        'Depth': 5, 'Durability Policy': 'Volatile', 'History Policy': 'Keep Last',
        'Reliability Policy': 'Reliable', 'Value': '/cbf_qp/h1_corridor',
    },
    'Value': True,
}
# Nothing in this workspace subscribes to /clicked_point or /move_base_simple/goal.
UNUSED_TOOLS = {
    'rviz_default_plugins/Select', 'rviz_default_plugins/SetGoal',
    'rviz_default_plugins/PublishPoint',
}


def restyle(manager):
    for display in manager.get('Displays', []):
        display.update(SCENE.get(display.get('Name'), {}))
    manager.setdefault('Global Options', {})['Background Color'] = BACKGROUND
    manager['Tools'] = [t for t in manager.get('Tools', []) if t.get('Class') not in UNUSED_TOOLS]


def with_panel(source, themed):
    """localization.rviz + the panel, written next to the panel's scripts."""
    with open(source, encoding='utf-8') as f:
        config = yaml.safe_load(f)
    panels = config.setdefault('Panels', [])
    if not any(p.get('Class') == PANEL['Class'] for p in panels):
        panels.insert(0, dict(PANEL))
    # A panel missing from the saved window state lands in the left dock, which
    # localization.rviz hides.
    geometry = config.setdefault('Window Geometry', {})
    geometry['Hide Left Dock'] = False
    geometry[PANEL['Name']] = {'collapsed': False}
    displays = config.setdefault('Visualization Manager', {}).setdefault('Displays', [])
    if not any(d.get('Name') == H1_CORRIDOR['Name'] for d in displays):
        after = [i for i, d in enumerate(displays) if d.get('Name') == 'TrackedPath']
        displays.insert(after[0] + 1 if after else len(displays), dict(H1_CORRIDOR))
    if themed:
        restyle(config.setdefault('Visualization Manager', {}))

    out_dir = os.path.join(os.environ.get('XDG_RUNTIME_DIR') or tempfile.gettempdir(), 'cart_panel')
    os.makedirs(out_dir, mode=0o700, exist_ok=True)
    out = os.path.join(out_dir, 'golf_cart.rviz')
    with open(out, 'w', encoding='utf-8') as f:
        yaml.safe_dump(config, f, sort_keys=False, allow_unicode=True, width=100000)
    return out


def as_bool(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ('true', '1', 'yes')


def launch_setup(context):
    themed = as_bool(context, 'theme')
    if as_bool(context, 'light'):
        config = os.path.join(
            get_package_share_directory('cart_rviz_panel'), 'rviz', 'cart_panel.rviz')
    else:
        config = with_panel(os.path.join(
            get_package_share_directory('lidar_localization_ros2'), 'rviz', 'localization.rviz'),
            themed)
    return [Node(
        package='rviz2',
        executable='rviz2',
        name='cart_panel_rviz',
        arguments=['-d', config],
        # Read by the panel: dark look for this RViz only.
        additional_env={'CART_RVIZ_THEME': 'dark'} if themed else {},
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'theme', default_value='true',
            description='true = ธีมมืดและฉากที่อ่านง่าย | false = หน้าตาเดิมของ localization.rviz',
        ),
        DeclareLaunchArgument(
            'light', default_value='false',
            description='true = rviz ตัวเบา มีแค่ panel ปุ่ม (ไม่ใช้ localization.rviz)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
