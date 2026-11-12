import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# รันทั้งสายควบคุมตามไดอะแกรมในสไลด์ CBF-QP:
#   nav_path.py (/nav_path, v_des)  ->  pagard (Path Error, Pure Pursuit, Speed Command,
#   Safety Supervisor, Command Check)  <->  cbf_qp_node.py (CBF-QP Solver: v*, L_d*)
# ทุกตัวรันจากไฟล์ใน src ตรง ๆ เหมือน nav_path.launch.py — แก้แล้วไม่ต้อง colcon build ใหม่
# ไม่รวม localization / low_level: สองอย่างนั้นยังเปิดตามขั้นตอนเดิม (start_cart.sh ฯลฯ)
SIM_DIR = '/home/myuser/2Localize_get_path/src/sim'
PKG_DIR = '/home/myuser/2Localize_get_path/src/nav_path_launch'

# ยึดที่ตัว interpreter python3 แบบเดียวกับ start_cart.sh กันไปจับ shell ที่แค่มีชื่อไฟล์อยู่ในคำสั่ง
PY = r'^[^ ]*python3[^ ]* [^ ]*'


def is_running(script_name):
    pattern = PY + script_name.replace('.', r'\.')
    return subprocess.run(['pgrep', '-f', pattern], capture_output=True).returncode == 0


def as_bool(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ('true', '1', 'yes')


# สวิตช์/ค่าของ cbf_qp_node ที่ตั้งจากปุ่มใน RViz panel ได้ (ว่าง = ตาม params_file)
SOLVER_SWITCHES = ('use_h1', 'use_h2', 'pure_pursuit_only')
SOLVER_VALUES = ('e_max', 'ay_max', 'fixed_ld')


def solver_overrides(context):
    """-> (dict ของค่าที่ให้มา, ข้อความผิดพลาดหรือ None)"""
    overrides = {}
    for name in SOLVER_SWITCHES:
        text = LaunchConfiguration(name).perform(context).strip().lower()
        if not text:
            continue
        if text not in ('true', 'false'):
            return {}, f'{name}:={text} ต้องเป็น true หรือ false'
        overrides[name] = text == 'true'
    for name in SOLVER_VALUES:
        text = LaunchConfiguration(name).perform(context).strip()
        if not text:
            continue
        try:
            overrides[name] = float(text)
        except ValueError:
            return {}, f'{name}:={text} ต้องเป็นตัวเลข'
    return overrides, None


def mode_text(overrides):
    if overrides.get('pure_pursuit_only'):
        ld = overrides.get('fixed_ld')
        return 'Pure Pursuit ล้วน' + (f' L_d={ld:g} m' if ld is not None else '')
    off = [name for name in ('use_h1', 'use_h2') if overrides.get(name) is False]
    return 'ปิด ' + ', '.join(off) if off else ''


def launch_setup(context):
    use_nav_path = as_bool(context, 'nav_path')
    use_controller = as_bool(context, 'controller')
    speed_source = LaunchConfiguration('speed_source').perform(context).strip().lower()
    if speed_source not in ('qp', 'profile'):
        return [EmitEvent(event=Shutdown(
            reason=f"speed_source='{speed_source}' ไม่รู้จัก ใช้ได้แค่ qp หรือ profile"))]

    # node ซ้ำ = /cmd_vel หรือ /adaptive_ld มีสอง publisher แย่งกัน -> ไม่เริ่มเลยดีกว่า
    conflicts = []
    if is_running('adaptive_ld_node_fixed.py'):
        conflicts.append('adaptive_ld_node_fixed.py ยังรันอยู่ (ตัวแก้ QP ตัวเดิม) — ปิดก่อน')
    if is_running('cbf_qp_node.py'):
        conflicts.append('cbf_qp_node.py รันอยู่แล้ว')
    use_logger = as_bool(context, 'log')
    if use_logger and is_running('cbf_qp_logger.py'):
        conflicts.append('cbf_qp_logger.py รันอยู่แล้ว — ปิดก่อน หรือใส่ log:=false')
    if use_controller and is_running('pagard_adaptive_fixed.py'):
        conflicts.append('pagard_adaptive_fixed.py รันอยู่แล้ว — ปิดก่อน หรือใส่ controller:=false')
    if use_controller and is_running('pose_speed_player.py'):
        conflicts.append('pose_speed_player.py รันอยู่ (ส่ง /cmd_vel เส้นเดียวกับ pagard) — ปิดก่อน')
    if use_nav_path and is_running('nav_path.py'):
        conflicts.append('nav_path.py รันอยู่แล้ว — ปิดก่อน หรือใส่ nav_path:=false')
    if conflicts:
        return [
            LogInfo(msg='!! ไม่เริ่ม cbf_qp.launch.py:\n   - ' + '\n   - '.join(conflicts)),
            EmitEvent(event=Shutdown(reason='พบ node ซ้ำ')),
        ]

    # ว่าง = nav_path.py ใช้ csv_path ที่ประกาศไว้ในตัวเอง เหมือนรันด้วยมือ
    csv_path = os.path.expanduser(LaunchConfiguration('csv_path').perform(context).strip())
    if csv_path and use_nav_path and not os.path.isfile(csv_path):
        return [
            LogInfo(msg=f'!! ไม่เริ่ม cbf_qp.launch.py: ไม่พบไฟล์เส้นทาง csv_path:={csv_path}'),
            EmitEvent(event=Shutdown(reason='ไม่พบไฟล์เส้นทาง')),
        ]

    overrides, problem = solver_overrides(context)
    if problem:
        return [
            LogInfo(msg=f'!! ไม่เริ่ม cbf_qp.launch.py: {problem}'),
            EmitEvent(event=Shutdown(reason='ค่าพารามิเตอร์ผิด')),
        ]

    qp_speed = speed_source == 'qp'
    actions = []
    if qp_speed and not use_controller:
        actions.append(LogInfo(msg=(
            '!! controller:=false + speed_source:=qp: pagard ที่รันเองต้องตั้ง '
            'speed_from_path_profile_only=False, cbf_speed_mode=replace, '
            'speed_profile_passthrough=False ไม่งั้น v* จะไม่ถูกนำไปใช้ '
            '(หรือใช้ speed_source:=profile)'
        )))

    if csv_path and not use_nav_path:
        actions.append(LogInfo(msg=(
            '!! nav_path:=false: csv_path ไม่ถูกใช้ เส้นทางคือของ nav_path.py ที่รันอยู่เอง'
        )))

    if use_nav_path:
        actions.append(Node(
            executable='/usr/bin/python3',
            arguments=[os.path.join(SIM_DIR, 'nav_path.py')],
            parameters=[{'csv_path': csv_path}] if csv_path else [],
            output='screen',
        ))

    if use_controller:
        # pagard ใช้ค่า default ของตัวเองทั้งหมด ยกเว้นที่จำเป็นต่อการให้ v* คุมความเร็ว
        controller_params = {}
        if qp_speed:
            controller_params.update({
                # ความเร็วมาจาก v* ของ QP (ซึ่ง ≤ v_des ของ CSV เสมอ) แทน CSV ตรง ๆ
                'speed_from_path_profile_only': False,
                'cbf_speed_mode': 'replace',
                # v* กระโดดได้เมื่อโค้งเข้ามาในระยะมอง -> ต้องมี rate limit + feedback ของ
                # pagard ไล่ตาม (บล็อก Speed Command ในไดอะแกรม)
                'speed_profile_passthrough': False,
            })
        if as_bool(context, 'dry_run'):
            controller_params['dry_run'] = True      # Twist ไปออก /cmd_vel_preview รถไม่ขยับ
        actions.append(Node(
            executable='/usr/bin/python3',
            arguments=[os.path.join(SIM_DIR, 'pagard_adaptive_fixed.py')],
            parameters=[controller_params] if controller_params else [],
            output='screen',
        ))

    actions.append(Node(
        executable='/usr/bin/python3',
        arguments=[os.path.join(PKG_DIR, 'nav_path_launch', 'cbf_qp_node.py')],
        parameters=[
            LaunchConfiguration('params_file').perform(context),
            # profile: ความเร็วจริงคือ v_des ของ CSV -> ให้ QP เลือก L_d เทียบกับความเร็วนั้น
            # CSV ภายในของ solver ปิดเสมอ: ผลการวิ่งเก็บเมื่อกดปุ่ม Logger CSV (หรือ log:=true) เท่านั้น
            {'optimize_speed': qp_speed, 'enable_csv_log': False},
            # ค่าที่ให้มาทาง launch argument (ปุ่มใน RViz panel) ทับค่าใน params_file
            *([overrides] if overrides else []),
        ],
        output='screen',
    ))

    if use_logger:
        # เก็บผลการวิ่งทั้งสายลง ~/controller_logs/<ปี-เดือน-วัน>/cbf_qp_run_<วันเวลา>.csv
        actions.append(Node(
            executable='/usr/bin/python3',
            arguments=[os.path.join(PKG_DIR, 'nav_path_launch', 'cbf_qp_logger.py')],
            output='screen',
        ))

    if as_bool(context, 'tune'):
        # หน้าต่างสไลเดอร์: เลือก cbf_qp_solver ในรายการด้านซ้าย แล้วลากปรับได้ตอนรถวิ่ง
        actions.append(Node(package='rqt_reconfigure', executable='rqt_reconfigure'))

    actions.insert(0, LogInfo(msg=(
        'CBF-QP: speed_source=' + speed_source
        + (' (v* จาก QP คุมความเร็ว)' if qp_speed else ' (ความเร็วจาก CSV, QP เลือกแต่ L_d)')
        + (' | DRY RUN: คำสั่งออก /cmd_vel_preview' if as_bool(context, 'dry_run') else '')
        + (' | เส้นทาง: ' + csv_path if csv_path and use_nav_path else '')
        + (' | สมการ: ' + mode_text(overrides) if mode_text(overrides) else '')
    )))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            # profile เป็นค่าเริ่มต้นตั้งแต่ 2026-10-12: รถวิ่งด้วยความเร็วที่อยู่ในไฟล์เส้นทาง
            # (คอลัมน์ v_target) ตรง ๆ ทุกเส้นทาง ไม่มี v* ของ QP, rate limit หรือเพดานมาคั่น
            'speed_source', default_value='profile',
            description='profile = ความเร็วจากไฟล์เส้นทาง (v_target) ตรง ๆ, QP เลือกแต่ L_d | '
                        'qp = v* ของ CBF-QP คุมความเร็ว (ตามสไลด์)',
        ),
        DeclareLaunchArgument(
            'dry_run', default_value='false',
            description='true = pagard ส่งคำสั่งไป /cmd_vel_preview รถจริงไม่ขยับ',
        ),
        DeclareLaunchArgument(
            # false ตั้งแต่ 2026-11-02: กดปุ่ม CBF-QP แล้ว Logger CSV ไม่เริ่มตาม ต้องกดของมันเอง
            'log', default_value='false',
            description='true = รัน cbf_qp_logger.py เก็บ CSV ลง ~/controller_logs/<ปี-เดือน-วัน>',
        ),
        DeclareLaunchArgument(
            'tune', default_value='false',
            description='true = เปิดหน้าต่างสไลเดอร์ (rqt_reconfigure) สำหรับปรับค่า cbf_qp_solver',
        ),
        DeclareLaunchArgument(
            'csv_path', default_value='',
            description='ไฟล์เส้นทางที่ส่งให้ nav_path.py | '
                        'ว่าง = ค่า csv_path ใน nav_path.py เอง (loop1_qp_comfort.csv)',
        ),
        DeclareLaunchArgument(
            'nav_path', default_value='true',
            description='false = ไม่รัน nav_path.py (รันเองอยู่แล้ว)',
        ),
        DeclareLaunchArgument(
            'controller', default_value='true',
            description='false = ไม่รัน pagard_adaptive_fixed.py (รันเองอยู่แล้ว)',
        ),
        *[
            DeclareLaunchArgument(
                name, default_value='',
                description=f'{name} ของ cbf_qp_node (true/false) | ว่าง = ตาม params_file',
            )
            for name in SOLVER_SWITCHES
        ],
        *[
            DeclareLaunchArgument(
                name, default_value='',
                description=f'{name} ของ cbf_qp_node (ตัวเลข) | ว่าง = ตาม params_file',
            )
            for name in SOLVER_VALUES
        ],
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(PKG_DIR, 'config', 'cbf_qp_preset.yaml'),
            description='พารามิเตอร์ของ cbf_qp_node.py',
        ),
        OpaqueFunction(function=launch_setup),
    ])
