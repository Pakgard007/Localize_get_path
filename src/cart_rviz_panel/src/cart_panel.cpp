#include "cart_rviz_panel/cart_panel.hpp"

#include <signal.h>
#include <unistd.h>

#include <QApplication>
#include <QDateTime>
#include <QDir>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFile>
#include <QFileInfo>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QPalette>
#include <QPointer>
#include <QProcess>
#include <QPushButton>
#include <QSaveFile>
#include <QScrollArea>
#include <QScrollBar>
#include <QSet>
#include <QSettings>
#include <QSignalBlocker>
#include <QShowEvent>
#include <QStandardPaths>
#include <QStorageInfo>
#include <QStyle>
#include <QStyleFactory>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace cart_rviz_panel
{

namespace
{

// Same anchor as start_cart.sh and cbf_qp.launch.py: the python3 interpreter
// followed by the script, so a shell whose command line merely mentions the
// script (the terminal's own bash, an editor, grep) never counts.
const QString kPy = QStringLiteral(R"(^[^ ]*python3[^ ]* [^ ]*)");
const QString kWorkspace = QStringLiteral("~/2Localize_get_path");

constexpr int kRefreshMs = 1000;
constexpr qint64 kStartWaitMs = 15000;  // terminal start-up + ros2 launch import
constexpr qint64 kStopWaitMs = 5000;    // before offering a harder signal

// The launch argument that names the map, as it shows in a command line.
const QString kMapArg = QStringLiteral(" map_path:=");
// The launch argument that names the route, as it shows in a command line.
const QString kRouteArg = QStringLiteral(" csv_path:=");
// Fewer points than this is not a path nav_path.py can head along.
constexpr int kRouteMinRows = 2;
// Route chips, one to a row; files beyond this are left out, to keep the
// column short.
constexpr size_t kMaxRoutes = 5;
// A path longer than this goes on a line of its own under its folder.
constexpr int kWhereLineChars = 34;
// A route on its own map scores 0.99-1.0 in routeOnMap(); the route of
// GlobalMapLoop1 on GlobalMapLoop2 scored 0.26 (2026-10-08).
constexpr double kRouteOnMapMin = 0.9;
constexpr double kRouteCellM = 1.0;
constexpr qint64 kRouteMapStride = 8;   // every 8th map point is plenty for 1 m cells
// A recording this long (6 m at get_path.py's 0.3 m a point) is kept when the
// recorder is started again; a shorter one is a recorder that was only tried.
constexpr int kKeepRows = 20;
// The bags of 2026-10-13 grew 2.2 MB/s with the cart driving (8 GB an hour).
// Below this much free disk, about two and a half hours, the card says so.
constexpr qint64 kBagLowBytes = 20LL * 1000 * 1000 * 1000;
// The controller's solver, whose switches the CBF-QP card sets.
const char * const kSolverNode = "/cbf_qp_solver";
// A reading of its parameters not answered by then is given up (the solver
// was stopped while it was asked).
constexpr qint64 kSolverReadWaitMs = 3000;
// How long the solver's reason for refusing a value stays on the card.
constexpr qint64 kSolverRefusalMs = 5000;

// Widgets are picked by object name and a "state" (run, busy, ended, off) or
// "kind" (start, stop, force) property, so the confirm dialogs, which inherit
// this sheet, keep their normal look. @name@ is a colour from kTokens.
const char * const kStyle = R"(
QLabel#title { color: palette(window-text); font-size: 14px; font-weight: 600; }
QLabel#section { color: @muted@; font-size: 11px; font-weight: 600; }
QLabel#chip { border-radius: 9px; padding: 1px 8px; font-size: 11px; font-weight: 600; }
QLabel#chip[tone="run"] { color: @greenText@; background: @greenSoft@; }
QLabel#chip[tone="busy"] { color: @amberText@; background: @amberSoft@; }
QLabel#chip[tone="ended"] { color: @redText@; background: @redSoft@; }
QLabel#chip[tone="off"] { color: @muted@; background: @neutralSoft@; }
QFrame#segment { border-radius: 2px; background: @segmentOff@; }
QFrame#segment[state="run"] { background: #22c55e; }
QFrame#segment[state="busy"] { background: #f59e0b; }
QFrame#segment[state="ended"] { background: #ef4444; }
QFrame#card { background: @card@; border: 1px solid @cardBorder@; border-radius: 10px; }
QFrame#card[state="busy"] { border-color: @amberBorder@; }
QFrame#card[state="ended"] { border-color: @redBorder@; }
QLabel#badge {
  border-radius: 12px; color: white; font-size: 12px; font-weight: 700; background: #16a34a;
}
QLabel#badge[state="busy"] { background: #d97706; }
QLabel#badge[state="ended"] { background: #dc2626; }
QLabel#badge[state="off"] { color: @muted@; background: @badgeOff@; }
QLabel#name { color: palette(text); font-size: 13px; font-weight: 600; }
QLabel#status { color: @muted@; font-size: 11px; }
QLabel#status[state="run"] { color: @greenText@; }
QLabel#status[state="busy"] { color: @amberText@; }
QLabel#status[state="ended"] { color: @redText@; }
QPushButton#action {
  min-width: 72px; min-height: 28px; max-height: 28px; padding: 0 10px;
  border-radius: 14px; font-size: 12px; font-weight: 600;
}
QPushButton#action[kind="start"] { color: white; background: @greenBtn@; border: none; }
QPushButton#action[kind="start"]:hover { background: @greenBtnHover@; }
QPushButton#action[kind="stop"] {
  color: palette(text); background: transparent; border: 1px solid @stopBorder@;
}
QPushButton#action[kind="stop"]:hover {
  color: @redText@; background: @redSoft@; border-color: @redBorder@;
}
QPushButton#action[kind="force"] { color: white; background: @redBtn@; border: none; }
QPushButton#action[kind="force"]:hover { background: @redBtnHover@; }
QPushButton#action:disabled { color: @muted@; background: @neutralSoft@; border: none; }
QPushButton#layer, QPushButton#map, QPushButton#route, QPushButton#knob {
  min-height: 24px; max-height: 24px; padding: 0 12px; border-radius: 12px; font-size: 12px;
  color: @muted@; background: transparent; border: 1px solid @cardBorder@;
}
QPushButton#layer:hover, QPushButton#map:hover, QPushButton#route:hover, QPushButton#knob:hover {
  border-color: @accentBorder@;
}
QPushButton#layer:checked, QPushButton#map:checked, QPushButton#route:checked,
QPushButton#knob:checked {
  color: @accentText@; background: @accentSoft@; border-color: @accentBorder@;
}
QPushButton#map, QPushButton#route, QPushButton#knob {
  min-height: 22px; max-height: 22px; font-weight: 600;
}
QPushButton#map:disabled, QPushButton#route:disabled, QPushButton#knob:disabled {
  color: @muted@; border-color: @cardBorder@;
}
QPushButton#map:checked:disabled, QPushButton#route:checked:disabled { color: @accentText@; }
QPushButton#knob:checked:disabled { color: @muted@; background: @neutralSoft@; }
QPushButton#knob { padding: 0 8px; }
QDoubleSpinBox#knob { min-height: 22px; max-height: 22px; font-size: 12px; }
QPushButton#knob[idle="true"], QPushButton#knob[idle="true"]:checked {
  color: @muted@; background: transparent; border: 1px dashed @cardBorder@;
}
QPushButton#knob[idle="true"]:checked { background: @neutralSoft@; }
QDoubleSpinBox#knob[idle="true"] { color: @muted@; }
QLabel#where[tone="warn"] { color: @amberText@; }
QLabel#hint, QLabel#where { color: @muted@; font-size: 11px; }
)";

// Colours that differ between a light and a dark RViz palette; the neutral
// ones come from the palette itself.
const struct
{
  const char * name;
  const char * light;
  const char * dark;
} kTokens[] = {
  {"card", "palette(base)", "#22262d"},
  {"cardBorder", "rgba(127, 127, 127, 0.28)", "rgba(255, 255, 255, 0.08)"},
  {"muted", "#868c96", "#8b94a1"},
  {"neutralSoft", "rgba(127, 127, 127, 0.15)", "rgba(255, 255, 255, 0.07)"},
  {"segmentOff", "rgba(127, 127, 127, 0.28)", "rgba(255, 255, 255, 0.12)"},
  {"badgeOff", "rgba(127, 127, 127, 0.18)", "rgba(255, 255, 255, 0.08)"},
  {"stopBorder", "rgba(127, 127, 127, 0.45)", "rgba(255, 255, 255, 0.18)"},
  {"greenText", "#15803d", "#4ade80"},
  {"greenSoft", "rgba(22, 163, 74, 0.13)", "rgba(74, 222, 128, 0.14)"},
  {"greenBtn", "#16a34a", "#16a34a"},
  {"greenBtnHover", "#15803d", "#22c55e"},
  {"amberText", "#b45309", "#fbbf24"},
  {"amberSoft", "rgba(217, 119, 6, 0.15)", "rgba(251, 191, 36, 0.14)"},
  {"amberBorder", "rgba(217, 119, 6, 0.6)", "rgba(245, 158, 11, 0.55)"},
  {"redText", "#dc2626", "#f87171"},
  {"redSoft", "rgba(220, 38, 38, 0.12)", "rgba(248, 113, 113, 0.14)"},
  {"redBorder", "rgba(220, 38, 38, 0.6)", "rgba(248, 113, 113, 0.5)"},
  {"redBtn", "#dc2626", "#dc2626"},
  {"redBtnHover", "#b91c1c", "#ef4444"},
  {"accentText", "#1d4ed8", "#93c5fd"},
  {"accentSoft", "rgba(59, 130, 246, 0.12)", "rgba(59, 130, 246, 0.2)"},
  {"accentBorder", "rgba(59, 130, 246, 0.5)", "rgba(96, 165, 250, 0.55)"},
};

QString panelStyle(bool dark)
{
  QString css = QString::fromUtf8(kStyle);
  for (const auto & token : kTokens) {
    css.replace(QString("@%1@").arg(token.name), dark ? token.dark : token.light);
  }
  return css;
}

// The rest of the window in the cart's own RViz: menus, tool bar, status bar,
// scroll bars and plain buttons. Inputs and trees are left to Fusion and the
// palette, which draw them well and keep their arrows and check marks.
const char * const kWindowStyle = R"(
QToolTip { color: #e3e6ea; background: #2a2f37; border: 1px solid #3a404a; padding: 4px 6px; }
QMenuBar { background: #15171b; color: #c9ced6; }
QMenuBar::item { padding: 4px 10px; background: transparent; }
QMenuBar::item:selected { background: #262a31; border-radius: 4px; }
QMenu { background: #1f2329; color: #e3e6ea; border: 1px solid #30353d; padding: 4px; }
QMenu::item { padding: 5px 22px 5px 12px; border-radius: 4px; }
QMenu::item:selected { background: #2f6fde; color: white; }
QMenu::separator { height: 1px; background: #30353d; margin: 4px 6px; }
QToolBar { background: #15171b; border: none; border-bottom: 1px solid #262a31; padding: 4px; spacing: 4px; }
QToolBar QToolButton { color: #c9ced6; padding: 5px 10px; border-radius: 6px; }
QToolBar QToolButton:hover { background: #262a31; color: white; }
QToolBar QToolButton:checked { background: #2f6fde; color: white; }
QStatusBar { background: #15171b; color: #8b94a1; border-top: 1px solid #262a31; }
QStatusBar QLabel { color: #8b94a1; }
QHeaderView::section {
  background: #1f2329; color: #8b94a1; border: none; border-right: 1px solid #262a31; padding: 4px 6px;
}
QScrollBar:vertical { background: transparent; width: 10px; margin: 0; }
QScrollBar::handle:vertical { background: #3a404a; border-radius: 4px; min-height: 28px; margin: 2px; }
QScrollBar:horizontal { background: transparent; height: 10px; margin: 0; }
QScrollBar::handle:horizontal { background: #3a404a; border-radius: 4px; min-width: 28px; margin: 2px; }
QScrollBar::handle:hover { background: #4a515c; }
QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; }
QScrollBar::add-page, QScrollBar::sub-page { background: none; }
QMainWindow::separator { background: #121418; width: 4px; height: 4px; }
QTabBar::tab { background: #15171b; color: #8b94a1; padding: 6px 14px; border: none; }
QTabBar::tab:selected { color: #e3e6ea; background: #1b1e23; border-bottom: 2px solid #2f6fde; }
QPushButton {
  background: #262a31; color: #e3e6ea; border: 1px solid #343a43; border-radius: 6px; padding: 4px 12px;
}
QPushButton:hover { background: #2e333b; }
QPushButton:disabled { color: #6b7380; }
)";

// Dark Fusion for the whole RViz process, matching the dark 3D view. Only when
// cart_panel.launch.py asks for it (CART_RVIZ_THEME=dark): added to any other
// RViz, the panel leaves its look alone and follows its palette.
void applyDarkTheme()
{
  static bool applied = false;
  if (applied) {
    return;
  }
  applied = true;
  QApplication::setStyle(QStyleFactory::create("Fusion"));
  QPalette palette;
  palette.setColor(QPalette::Window, QColor(27, 30, 35));
  palette.setColor(QPalette::WindowText, QColor(227, 230, 234));
  palette.setColor(QPalette::Base, QColor(20, 22, 26));
  palette.setColor(QPalette::AlternateBase, QColor(26, 29, 34));
  palette.setColor(QPalette::Text, QColor(227, 230, 234));
  palette.setColor(QPalette::Button, QColor(38, 42, 49));
  palette.setColor(QPalette::ButtonText, QColor(227, 230, 234));
  palette.setColor(QPalette::BrightText, Qt::white);
  palette.setColor(QPalette::Highlight, QColor(47, 111, 222));
  palette.setColor(QPalette::HighlightedText, Qt::white);
  palette.setColor(QPalette::Link, QColor(110, 168, 254));
  palette.setColor(QPalette::ToolTipBase, QColor(42, 47, 55));
  palette.setColor(QPalette::ToolTipText, QColor(227, 230, 234));
  palette.setColor(QPalette::PlaceholderText, QColor(107, 115, 128));
  palette.setColor(QPalette::Light, QColor(58, 64, 74));
  palette.setColor(QPalette::Midlight, QColor(48, 53, 61));
  palette.setColor(QPalette::Mid, QColor(38, 42, 49));
  palette.setColor(QPalette::Dark, QColor(18, 20, 24));
  palette.setColor(QPalette::Shadow, QColor(11, 12, 14));
  for (const QPalette::ColorRole role : {QPalette::WindowText, QPalette::Text, QPalette::ButtonText}) {
    palette.setColor(QPalette::Disabled, role, QColor(107, 115, 128));
  }
  QApplication::setPalette(palette);
  qApp->setStyleSheet(kWindowStyle);
}

bool matches(const QRegularExpression & re, const QString & cmd)
{
  return !re.pattern().isEmpty() && re.match(cmd).hasMatch();
}

// Sets a property the style sheet selects on, re-polishing only on a change.
void restyle(QWidget * widget, const char * name, const QString & value)
{
  if (widget->property(name).toString() == value) {
    return;
  }
  widget->setProperty(name, value);
  widget->style()->unpolish(widget);
  widget->style()->polish(widget);
}

// Seconds since the process started, or -1 if it is gone.
qint64 secondsRunning(pid_t pid)
{
  QFile stat_file(QStringLiteral("/proc/%1/stat").arg(pid));
  QFile uptime_file(QStringLiteral("/proc/uptime"));
  if (!stat_file.open(QIODevice::ReadOnly) || !uptime_file.open(QIODevice::ReadOnly)) {
    return -1;
  }
  // Fields counted from the one after the ")" closing the command name, which
  // may contain spaces: index 19 is field 22, the start time in clock ticks.
  const QByteArray stat = stat_file.readAll();
  const QList<QByteArray> fields = stat.mid(stat.lastIndexOf(')') + 2).split(' ');
  if (fields.size() < 20) {
    return -1;
  }
  const double uptime = uptime_file.readAll().split(' ').value(0).toDouble();
  return static_cast<qint64>(uptime - fields[19].toDouble() / sysconf(_SC_CLK_TCK));
}

QString formatUptime(qint64 seconds)
{
  if (seconds < 60) {
    return "เพิ่งเริ่ม";
  }
  const qint64 minutes = seconds / 60;
  if (minutes < 60) {
    return QString("%1 นาที").arg(minutes);
  }
  return QString("%1 ชม. %2 นาที").arg(minutes / 60).arg(minutes % 60);
}

QString workspaceDir()
{
  return QDir::homePath() + kWorkspace.mid(1);
}

// As one word for the shell; left bare when it needs no quoting, so the
// command reads the same in the terminal as it does typed by hand.
QString shellWord(const QString & text)
{
  static const QRegularExpression plain(QStringLiteral("^[A-Za-z0-9_./:=+-]+$"));
  if (plain.match(text).hasMatch()) {
    return text;
  }
  return "'" + QString(text).replace("'", "'\\''") + "'";
}

// A path written with ~, as in the scripts and on a command line.
QString expandHome(const QString & path)
{
  return path.startsWith("~/") ? QDir::homePath() + path.mid(1) : path;
}

// What a launch was given for an argument such as kMapArg: up to the next
// name:= argument, so a path with a space in it stays whole. Empty when the
// command line has no such argument.
QString argumentOf(const QString & cmd, const QString & arg)
{
  const QRegularExpression argument(
    QRegularExpression::escape(arg) + R"((.+?)(?= [A-Za-z_][A-Za-z0-9_]*:=|$))");
  const QRegularExpressionMatch match = argument.match(cmd);
  return match.hasMatch() ? match.captured(1) : QString();
}

// The route nav_path.py publishes when it is given no csv_path: the default in
// the script itself, so the check follows a change made there.
QString routePath()
{
  const QString dir = workspaceDir() + "/src/sim";
  QFile script(dir + "/nav_path.py");
  if (!script.open(QIODevice::ReadOnly)) {
    return QString();
  }
  static const QRegularExpression csv(
    R"(declare_parameter\(\s*'csv_path'\s*,\s*os\.path\.join\(\s*SCRIPT_DIR\s*,\s*'([^']+)'\s*\))");
  const QRegularExpressionMatch match = csv.match(QString::fromUtf8(script.readAll()));
  return match.hasMatch() ? dir + "/" + match.captured(1) : QString();
}

// The file get_path.py records into when it is given no file_path: the default
// in the script itself. Empty when it cannot be told.
QString recorderFile()
{
  QFile script(workspaceDir() + "/src/sim/get_path.py");
  if (!script.open(QIODevice::ReadOnly)) {
    return QString();
  }
  static const QRegularExpression file(
    R"(declare_parameter\(\s*'file_path'\s*,\s*(?:os\.path\.expanduser\(\s*)?'([^']+)')");
  const QRegularExpressionMatch match = file.match(QString::fromUtf8(script.readAll()));
  return match.hasMatch() ? expandHome(match.captured(1)) : QString();
}

// The file cbf_qp_logger.py starts on every run when it is given no csv_path,
// as the strftime pattern in the script itself. Empty when it cannot be told.
QString loggerPattern()
{
  QFile script(workspaceDir() + "/src/nav_path_launch/nav_path_launch/cbf_qp_logger.py");
  if (!script.open(QIODevice::ReadOnly)) {
    return QString();
  }
  static const QRegularExpression file(
    R"(declare_parameter\(\s*'csv_path'\s*,[^']*strftime\(\s*'([^']+)')");
  const QRegularExpressionMatch match = file.match(QString::fromUtf8(script.readAll()));
  return match.hasMatch() ? expandHome(match.captured(1)) : QString();
}

// The newest file a strftime pattern has made, going by name: its numbers are
// written year first, so the last name is the latest. `parts` is the pattern
// below `dir`, one folder level each. Empty when there is none yet.
QString newestOf(const QString & dir, QStringList parts)
{
  const QString part = parts.takeFirst();
  const bool file = parts.isEmpty();
  if (!part.contains('%')) {
    const QString next = dir + "/" + part;
    return !file ? newestOf(next, parts) : QFileInfo(next).isFile() ? next : QString();
  }
  static const QRegularExpression directive(QStringLiteral("%[A-Za-z]"));
  const QStringList names = QDir(dir).entryList(
    QStringList{QString(part).replace(directive, "*")},
    file ? QDir::Files : QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name | QDir::Reversed);
  for (const QString & name : names) {
    const QString found = file ? dir + "/" + name : newestOf(dir + "/" + name, parts);
    if (!found.isEmpty()) {
      return found;
    }
  }
  return QString();
}

// The CSV a process has open: the one a logger is writing right now.
QString openCsv(pid_t pid)
{
  const QString dir = QStringLiteral("/proc/%1/fd").arg(pid);
  const QStringList fds =
    QDir(dir).entryList(QDir::AllEntries | QDir::System | QDir::NoDotAndDotDot);
  for (const QString & fd : fds) {
    const QString target = QFile::symLinkTarget(dir + "/" + fd);
    if (target.endsWith(".csv")) {
      return target;
    }
  }
  return QString();
}

// With the home directory written as ~, as in the commands.
QString tilde(const QString & path)
{
  const QString home = QDir::homePath();
  return path.startsWith(home + "/") ? "~" + path.mid(home.size()) : path;
}

// What a recorder's CSV holds: its waypoints, from the s_m column of the last
// one how far along the path that is, and from the v_target column the speed
// the cart had at them. A controller following the file reads that column as
// the speed to drive at.
struct Recorded
{
  int rows = 0;
  double metres = -1.0;
  bool xy = false;     // has the x and y columns of a path
  bool speed = false;  // has a speed in v_target
  double v_min = 0.0;
  double v_max = 0.0;
  double v_last = 0.0;
};

Recorded recorded(const QString & path)
{
  Recorded result;
  QFile file(path);
  // A 500 m loop is 160 kB; anything huge is not a recording of this kind.
  if (path.isEmpty() || !file.open(QIODevice::ReadOnly) || file.size() > 20 * 1000 * 1000) {
    return result;
  }
  const QList<QByteArray> lines = file.readAll().split('\n');
  const QList<QByteArray> header = lines.value(0).trimmed().split(',');
  const int s_column = header.indexOf("s_m");
  const int v_column = header.indexOf("v_target");
  result.xy = header.contains("x") && header.contains("y");
  QByteArray last;
  for (int i = 1; i < lines.size(); ++i) {
    if (lines[i].trimmed().isEmpty()) {
      continue;
    }
    ++result.rows;
    last = lines[i];
    bool ok = false;
    const double v =
      v_column < 0 ? 0.0 : lines[i].split(',').value(v_column).trimmed().toDouble(&ok);
    if (ok && std::isfinite(v)) {
      result.v_min = result.speed ? std::min(result.v_min, v) : v;
      result.v_max = result.speed ? std::max(result.v_max, v) : v;
      result.v_last = v;
      result.speed = true;
    }
  }
  if (result.rows > 0 && s_column >= 0) {
    bool ok = false;
    const double metres = last.trimmed().split(',').value(s_column).toDouble(&ok);
    result.metres = ok ? metres : -1.0;
  }
  return result;
}

QString describe(int rows, double metres)
{
  QString text = QString("%L1 จุด").arg(rows);
  if (metres >= 0.0) {
    text += QString(" · %L1 ม.").arg(qRound(metres));
  }
  return text;
}

QString describe(const Recorded & done)
{
  return describe(done.rows, done.metres);
}

QString mps(double speed)
{
  return QString::number(speed, 'f', 1);
}

QString speedRange(double v_min, double v_max)
{
  return QString("%1–%2 m/s").arg(mps(v_min), mps(v_max));
}

QString bytesText(qint64 bytes)
{
  if (bytes < 1000 * 1000) {
    return QString("%L1 kB").arg((bytes + 999) / 1000);
  }
  if (bytes < 1000LL * 1000 * 1000) {
    return QString("%L1 MB").arg(qRound64(bytes / 1e6));
  }
  return QString("%1 GB").arg(bytes / 1e9, 0, 'f', 1);
}

QString durationText(qint64 seconds)
{
  if (seconds < 60) {
    return QString("%1 วินาที").arg(seconds);
  }
  if (seconds < 3600) {
    return QString("%1 นาที").arg(seconds / 60);
  }
  return QString("%1 ชม. %2 นาที").arg(seconds / 3600).arg(seconds / 60 % 60);
}

// Free space on the disk a folder is on, or on the home's while it does not
// exist yet. Negative when it cannot be told.
qint64 freeBytes(const QString & dir)
{
  const QStorageInfo disk(QFileInfo::exists(dir) ? dir : QDir::homePath());
  return disk.isValid() ? disk.bytesAvailable() : -1;
}

// The newest bag in a folder: a folder holding a metadata.yaml, or the storage
// file of one that was never closed. Empty when there is none.
QString newestBag(const QString & dir)
{
  if (dir.isEmpty()) {
    return QString();
  }
  const QFileInfoList folders =
    QDir(dir).entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Time);
  for (const QFileInfo & folder : folders) {
    const QStringList files = QDir(folder.absoluteFilePath()).entryList(
      QStringList{"metadata.yaml", "*.db3", "*.mcap"}, QDir::Files);
    if (!files.isEmpty()) {
      return folder.absoluteFilePath();
    }
  }
  return QString();
}

// What a bag folder holds. rosbag2 writes metadata.yaml when it is stopped with
// Ctrl+C; one killed, or cut off by a power loss, has none until
// `ros2 bag reindex` rebuilds it.
struct Bag
{
  qint64 bytes = 0;
  bool closed = false;
  qint64 seconds = -1;  // how long it is, from metadata.yaml
};

Bag bagAt(const QString & dir)
{
  Bag bag;
  if (dir.isEmpty()) {
    return bag;
  }
  for (const QFileInfo & file : QDir(dir).entryInfoList(QDir::Files)) {
    bag.bytes += file.size();
  }
  QFile metadata(dir + "/metadata.yaml");
  if (metadata.open(QIODevice::ReadOnly)) {
    bag.closed = true;
    // The first duration in the file is the whole bag's; each file's comes later.
    static const QRegularExpression duration(QStringLiteral(R"(duration:\s*nanoseconds:\s*(\d+))"));
    const QRegularExpressionMatch match = duration.match(QString::fromUtf8(metadata.readAll()));
    if (match.hasMatch()) {
      bag.seconds = match.captured(1).toLongLong() / 1000000000;
    }
  }
  return bag;
}

quint64 cellOf(double x, double y)
{
  const auto cx = static_cast<qint32>(std::floor(x / kRouteCellM));
  const auto cy = static_cast<qint32>(std::floor(y / kRouteCellM));
  return (static_cast<quint64>(static_cast<quint32>(cx)) << 32) | static_cast<quint32>(cy);
}

}  // namespace

double CartPanel::routeOnMap(const QString & map_path, const QString & route_path)
{
  QFile map(map_path);
  if (!map.open(QIODevice::ReadOnly)) {
    return -1.0;
  }
  // The PCD header says where x and y sit in a point and how long a point is.
  QList<QByteArray> fields, sizes, types, counts;
  qint64 points = 0;
  bool binary = false;
  while (!map.atEnd()) {
    const QList<QByteArray> words = map.readLine(4096).simplified().split(' ');
    const QByteArray key = words.value(0);
    if (key == "FIELDS") {
      fields = words.mid(1);
    } else if (key == "SIZE") {
      sizes = words.mid(1);
    } else if (key == "TYPE") {
      types = words.mid(1);
    } else if (key == "COUNT") {
      counts = words.mid(1);
    } else if (key == "POINTS") {
      points = words.value(1).toLongLong();
    } else if (key == "DATA") {
      binary = words.value(1) == "binary";
      break;
    }
  }
  qint64 point_size = 0;
  qint64 x_at = -1;
  qint64 y_at = -1;
  for (int i = 0; i < fields.size(); ++i) {
    const bool is_float = sizes.value(i) == "4" && types.value(i) == "F";
    if (fields[i] == "x" && is_float) {
      x_at = point_size;
    } else if (fields[i] == "y" && is_float) {
      y_at = point_size;
    }
    point_size += sizes.value(i).toLongLong() * std::max(counts.value(i, "1").toLongLong(), 1LL);
  }
  const qint64 start = map.pos();
  if (!binary || x_at < 0 || y_at < 0 || points <= 0 || start + points * point_size > map.size()) {
    return -1.0;
  }
  const uchar * data = map.map(start, points * point_size);
  if (!data) {
    return -1.0;
  }
  QSet<quint64> cells;
  for (qint64 i = 0; i < points; i += kRouteMapStride) {
    float x;
    float y;
    std::memcpy(&x, data + i * point_size + x_at, sizeof(x));
    std::memcpy(&y, data + i * point_size + y_at, sizeof(y));
    if (std::isfinite(x) && std::isfinite(y)) {
      cells.insert(cellOf(x, y));
    }
  }

  QFile route(route_path);
  if (!route.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return -1.0;
  }
  const QList<QByteArray> header = route.readLine().trimmed().split(',');
  const int x_column = header.indexOf("x");
  const int y_column = header.indexOf("y");
  if (x_column < 0 || y_column < 0) {
    return -1.0;
  }
  int total = 0;
  int on_map = 0;
  while (!route.atEnd()) {
    const QList<QByteArray> row = route.readLine().trimmed().split(',');
    bool x_ok = false;
    bool y_ok = false;
    const double x = row.value(x_column).toDouble(&x_ok);
    const double y = row.value(y_column).toDouble(&y_ok);
    if (!x_ok || !y_ok) {
      continue;
    }
    ++total;
    bool found = false;
    for (int dx = -1; dx <= 1 && !found; ++dx) {
      for (int dy = -1; dy <= 1 && !found; ++dy) {
        found = cells.contains(cellOf(x + dx * kRouteCellM, y + dy * kRouteCellM));
      }
    }
    on_map += found;
  }
  return total > 0 ? static_cast<double>(on_map) / total : -1.0;
}

CartPanel::CartPanel(QWidget * parent)
: rviz_common::Panel(parent),
  script_dir_(QStandardPaths::writableLocation(QStandardPaths::RuntimeLocation) +
    QStringLiteral("/cart_panel"))
{
  addStep(
    "ouster", "LiDAR Ouster",
    "ros2 launch ouster_ros driver.launch.py",
    kPy + R"(ros2 launch ouster_ros driver\.launch\.py)");
  addStep(
    "localization", "Localization",
    "ros2 launch lidar_localization_ros2 lidar_localization.launch.py",
    kPy + R"(ros2 launch lidar_localization_ros2 lidar_localization\.launch\.py)");
  // start_cart.sh resets can0 and waits for the STM32 (sudo password and the
  // power-cycle prompt appear in its terminal), then execs the low_level launch.
  addStep(
    "lowlevel", "Low-level",
    "bash ~/2Localize_get_path/start_cart.sh",
    kPy + R"(ros2 launch golfcart_low_level )",
    R"(^[^ ]*bash [^ ]*start_cart\.sh)");
  addStep(
    "gps", "GPS u-blox",
    "ros2 launch ublox_gps ublox_gps_node_zed_f9r-launch.py",
    kPy + R"(ros2 launch ublox_gps ublox_gps_node_zed_f9r-launch\.py)");
  addStep(
    "ntrip", "NTRIP (RTK)",
    "ros2 launch ntrip_client ntrip_client-launch.py",
    kPy + R"(ros2 launch ntrip_client ntrip_client-launch\.py)");
  addStep(
    "cbf_qp", "CBF-QP",
    "ros2 launch nav_path_launch cbf_qp.launch.py tune:=true",
    kPy + R"(ros2 launch nav_path_launch cbf_qp\.launch\.py)");
  // Runs only when this row is pressed: cbf_qp.launch.py leaves it out by default
  // (log:=false). Started with log:=true by hand, this row shows it running
  // inside step 6 instead of offering a second copy.
  addStep(
    "logger", "Logger CSV",
    "/usr/bin/python3 ~/2Localize_get_path/src/nav_path_launch/nav_path_launch/cbf_qp_logger.py",
    kPy + R"(cbf_qp_logger\.py)");
  find("logger")->logs = loggerPattern();

  // Driven by hand with this running, the cart's poses and its speed at each
  // of them become the raw material of a route. It needs /pcl_pose, so the
  // localization; the speed is /speed_status, so the low level's, and without
  // that get_path.py works it out from how far the pose has moved.
  Step & recorder = addTool(
    "get_path", "เก็บ path + ความเร็ว",
    "/usr/bin/python3 ~/2Localize_get_path/src/sim/get_path.py",
    kPy + R"(get_path\.py)");
  recorder.records = recorderFile();
  recorder.needs = QStringList{"localization"};
  recorder.speed_from = "lowlevel";

  // Every topic but the LiDAR's clouds, images and packets, which are most of
  // its data, /navcov and /initial_map. With -a it also takes up topics that
  // appear later, so it can be started before the other steps. Ctrl+C closes
  // the bag. /initial_map is the map file itself (map_path); on FullGlobalMap
  // it is one 1.16 GB message, over sqlite's 1 GB, and the recorder died on
  // it ~5 s in without writing metadata.yaml (2026-11-11).
  Step & bag = addTool(
    "rosbag", "เก็บ ROSbag",
    "ros2 bag record -a -x \"/ouster/(points|.*_image|.*_packets|scan|telemetry)|/navcov|/initial_map\" "
    "-o ~/ROsBAg/my_bag_$(date +%Y%m%d_%H%M%S)",
    kPy + R"(ros2 bag record )");
  bag.bags = QDir::homePath() + "/ROsBAg";

  Step * cbf_qp = find("cbf_qp");
  cbf_qp->confirm_start =
    "CBF-QP จะสั่งรถให้วิ่งตาม path เอง\n"
    "เช็กก่อนว่า localization ล็อกตำแหน่งแล้ว และทางข้างหน้าว่าง\n\n"
    "เริ่มเลยไหม?";
  cbf_qp->needs = QStringList{"localization", "lowlevel"};
  cbf_qp->follows_route = true;
  Step * lowlevel = find("lowlevel");
  // The STM32 firmware only raises a flag when /speed_cmd stops arriving; it
  // keeps driving at the last target speed.
  lowlevel->confirm_stop =
    "หยุด low_level?\n\n"
    "ถ้ารถยังวิ่งอยู่ STM32 จะวิ่งต่อด้วยความเร็วสุดท้าย "
    "เพราะไม่หยุดเองเมื่อคำสั่งขาด ให้จอดรถให้นิ่งก่อน\n\n"
    "จะเปิดใหม่ให้ใช้ปุ่มนี้ (start_cart.sh) เสมอ";
  // What start_cart.sh is blocked on, told by the command it is running.
  lowlevel->waits = {
    {QRegularExpression("^sudo "), "รอรหัส sudo ใน terminal"},
    {QRegularExpression("^(timeout [^ ]+ )?candump "), "รอสัญญาณ STM32 · ดู terminal"},
  };
  // Without it the launch waits 10 s for the receiver and then dies, which
  // looked like the NTRIP step, started meanwhile, had killed it.
  find("gps")->device = "/dev/ttyACM0";

  // The experiment's switches of the CBF-QP solver, in pairs: a constraint and
  // the number it is tuned by, one row each. Ranges as in cbf_qp_node.py.
  auto flag = [](const char * name, const char * label, const char * tip) {
      Knob knob;
      knob.name = name;
      knob.label = label;
      knob.tip = tip;
      knob.flag = true;
      return knob;
    };
  auto number = [](
    const char * name, const char * label, const char * tip, double low, double high,
    double step, int decimals, const char * unit) {
      Knob knob;
      knob.name = name;
      knob.label = label;
      knob.tip = tip;
      knob.low = low;
      knob.high = high;
      knob.step = step;
      knob.decimals = decimals;
      knob.unit = unit;
      return knob;
    };
  knobs_ = {
    flag(
      "use_h1", "h₁ กรอบ",
      "h₁ = e_max² − e_lat²: ไม่ให้รถออกนอกกรอบกว้าง ±e_max จากเส้นทาง\n"
      "ปิด = ไม่ใช้ h₁ ตัดสิน L_d (ยังคำนวณไว้ดูใน log)"),
    number("e_max", "±", "e_max: ครึ่งความกว้างกรอบของ h₁", 0.1, 3.0, 0.05, 2, " ม."),
    flag(
      "use_h2", "h₂ เหวี่ยง",
      "h₂ = a_y,max − v²κ̂: เพดานแรงเหวี่ยงด้านข้าง\n"
      "ปิด = ไม่ใช้ h₂ ตัดสิน (ยังคำนวณไว้ดูใน log)"),
    number(
      "ay_max", "",
      "a_y,max: เพดานแรงเหวี่ยงของ h₂ ต่ำ = เข้าโค้งช้าลง\n"
      "ยังยืด L_d,ref ด้วย √(0.4/a_y,max) แม้ปิด h₂", 0.05, 5.0, 0.05, 2, " m/s²"),
    flag(
      "pure_pursuit_only", "Pure Pursuit",
      "เปิด = เหลือแค่ Pure Pursuit: δ = atan(2L·sinα / L_d) กับ L_d คงที่\n"
      "ไม่มี QP, h₁, h₂ และ pagard ไม่ใช้ heading blend / CTE correction / filter\n"
      "ความเร็วยังมาจากไฟล์เส้นทาง"),
    number("fixed_ld", "L_d ", "L_d คงที่ของโหมด Pure Pursuit", 1.0, 12.0, 0.5, 1, " ม."),
  };
  readPresets();

  // The displays an operator switches, by their names in localization.rviz.
  layers_ = {
    {"แผนที่", {"InitialMap", "Map"}},
    {"สแกนสด", {"LiveScan"}},
    {"เส้นทาง", {"TrackedPath", "GlobalPath", "GoalMarker", "LookaheadMarker", "CTELine"}},
    {"กรอบ h₁", {"H1Corridor"}},  // added by cart_panel.launch.py
    {"ตัวรถ", {"RobotModel", "EstimatedPose"}},
    {"กริด", {"Grid"}},
  };

  themed_ = qgetenv("CART_RVIZ_THEME") == "dark";
  if (themed_) {
    applyDarkTheme();
  }
  findMaps();
  // The route chosen last time; refreshRoutes() checks that it is still there.
  route_ = QSettings("cart_rviz_panel", "panel").value("route").toString();
  buildUi();

  auto * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &CartPanel::refresh);
  timer->start(kRefreshMs);
  refresh();
}

void CartPanel::addStep(
  const QString & key, const QString & name, const QString & command,
  const QString & running, const QString & starting)
{
  Step step;
  step.key = key;
  step.number = 1 + static_cast<int>(
    std::count_if(steps_.begin(), steps_.end(), [](const Step & other) {return !other.tool;}));
  step.name = name;
  step.label = QString("%1. %2").arg(step.number).arg(name);
  step.command = command;
  step.running.setPattern(running);
  step.starting.setPattern(starting);
  step.terminal.setPattern(
    "^[^ ]*bash .*" + QRegularExpression::escape(script_dir_ + "/" + key + ".sh") + "$");
  steps_.push_back(step);
}

// What cbf_qp_preset.yaml, the launch's default params_file, sets for each
// knob: what the solver starts with when the panel passes nothing.
void CartPanel::readPresets()
{
  QFile file(workspaceDir() + "/src/nav_path_launch/config/cbf_qp_preset.yaml");
  const QString text = file.open(QIODevice::ReadOnly) ? QString::fromUtf8(file.readAll()) : QString();
  // Defaults of cbf_qp_core.CbfQpParams, for a name the file does not set.
  static const QHash<QString, QVariant> builtin = {
    {"use_h1", true}, {"use_h2", true}, {"pure_pursuit_only", false},
    {"e_max", 0.8}, {"ay_max", 0.4}, {"fixed_ld", 4.0},
  };
  for (Knob & knob : knobs_) {
    knob.preset = builtin.value(knob.name);
    const QRegularExpression line(
      "^\\s*" + QRegularExpression::escape(knob.name) + "\\s*:\\s*([^\\s#]+)",
      QRegularExpression::MultilineOption);
    const QString value = line.match(text).captured(1).toLower();
    bool ok = false;
    const double number = value.toDouble(&ok);
    if (knob.flag && (value == "true" || value == "false")) {
      knob.preset = value == "true";
    } else if (!knob.flag && ok) {
      knob.preset = number;
    }
  }
}

// A card below the steps: started, watched and stopped the same way.
CartPanel::Step & CartPanel::addTool(
  const QString & key, const QString & name, const QString & command, const QString & running)
{
  addStep(key, name, command, running);
  Step & tool = steps_.back();
  tool.tool = true;
  tool.number = 0;
  tool.label = name;
  return tool;
}

// Every point cloud in the workspace's src is a map to choose from, so a new
// one shows up the next time RViz is opened. The choice is kept across RViz
// restarts; with none made yet it is the first by name.
void CartPanel::findMaps()
{
  const QFileInfoList files = QDir(workspaceDir() + "/src").entryInfoList(
    QStringList{"*.pcd"}, QDir::Files, QDir::Name);
  for (const QFileInfo & file : files) {
    Map map;
    map.path = file.absoluteFilePath();
    // GlobalMapLoop1.pcd -> Loop1: every map here is a renamed GlobalMap.pcd.
    static const QString prefix = QStringLiteral("GlobalMap");
    const QString name = file.completeBaseName();
    map.label = name.startsWith(prefix) && name.size() > prefix.size() ?
      name.mid(prefix.size()) : name;
    maps_.push_back(map);
  }
  const QString saved = QSettings("cart_rviz_panel", "panel").value("map").toString();
  if (mapAt(saved)) {
    map_ = saved;
  } else if (!maps_.empty()) {
    map_ = maps_.front().path;
  }
}

const CartPanel::Map * CartPanel::mapAt(const QString & path) const
{
  for (const Map & map : maps_) {
    if (map.path == path) {
      return &map;
    }
  }
  return nullptr;
}

void CartPanel::selectMap(const QString & path)
{
  map_ = path;
  QSettings("cart_rviz_panel", "panel").setValue("map", path);
}

void CartPanel::selectRoute(const QString & path)
{
  route_ = path;
  QSettings("cart_rviz_panel", "panel").setValue("route", path);
}

const CartPanel::Route * CartPanel::routeAt(const QString & path) const
{
  for (const Route & route : routes_) {
    if (route.path == path) {
      return &route;
    }
  }
  return nullptr;
}

// The route the controller is started on: the chosen one, or with none to
// choose from the default in nav_path.py.
QString CartPanel::routeToFollow() const
{
  return route_.isEmpty() ? routePath() : route_;
}

// The step's command line. The localization gets the chosen map; with no map
// found in src it runs on the one in localization.yaml, as it did before. The
// controller gets the chosen route the same way, and the solver's switches
// that were set here.
QString CartPanel::commandOf(const Step & step) const
{
  if (step.key == "localization" && !map_.isEmpty()) {
    return step.command + kMapArg + shellWord(map_);
  }
  if (step.follows_route) {
    const QString route = route_.isEmpty() ? QString() : kRouteArg + shellWord(route_);
    return step.command + route + knobArguments();
  }
  return step.command;
}

// Whether the localization is running, and the map its launch was given: empty
// when it was started without map_path (then it is the one in localization.yaml).
bool CartPanel::runningMap(const std::vector<Proc> & procs, QString * path)
{
  const Step * step = find("localization");
  for (const Proc & proc : procs) {
    if (matches(step->running, proc.cmd)) {
      *path = argumentOf(proc.cmd, kMapArg);
      return true;
    }
  }
  return false;
}

// Whether the controller's launch is running, and the route its nav_path.py
// was given: the default in nav_path.py when it was started without csv_path.
bool CartPanel::runningRoute(const std::vector<Proc> & procs, QString * path)
{
  const Step * step = find("cbf_qp");
  for (const Proc & proc : procs) {
    if (matches(step->running, proc.cmd)) {
      const QString given = argumentOf(proc.cmd, kRouteArg);
      *path = given.isEmpty() ? routePath() : QDir::cleanPath(expandHome(given));
      return true;
    }
  }
  return false;
}

const CartPanel::Sheet & CartPanel::sheet(const QString & path)
{
  const QFileInfo file(path);
  const QString stamp =
    QString("%1|%2").arg(file.size()).arg(file.lastModified().toMSecsSinceEpoch());
  Sheet & known = sheets_[path];
  if (known.stamp != stamp) {
    const Recorded done = recorded(path);
    known.stamp = stamp;
    known.route = done.xy && done.rows >= kRouteMinRows;
    known.rows = done.rows;
    known.metres = done.metres;
    known.speed = done.speed;
    known.v_min = done.v_min;
    known.v_max = done.v_max;
  }
  return known;
}

// The routes to choose from: every CSV of waypoints next to nav_path.py, one
// per loop. What a recorder writes is not among them: a recording becomes a
// route when it is moved there, over the loop's file. Read on every refresh,
// so the chips follow the files.
std::vector<CartPanel::Route> CartPanel::findRoutes(const QString & running)
{
  std::vector<Route> found;
  const QFileInfoList beside = QDir(workspaceDir() + "/src/sim").entryInfoList(
    QStringList{"*.csv"}, QDir::Files, QDir::Name);
  for (const QFileInfo & file : beside) {
    const QString path = file.absoluteFilePath();
    if (sheet(path).route) {
      Route route;
      route.path = path;
      route.label = file.completeBaseName();
      found.push_back(route);
    }
  }
  for (size_t i = found.size(); i-- > 0 && found.size() > kMaxRoutes; ) {
    if (found[i].path != route_ && found[i].path != running) {
      found.erase(found.begin() + static_cast<std::ptrdiff_t>(i));
    }
  }
  return found;
}

// Anything that steers the cart by its pose. A new map is a new frame: the
// pose would jump while it drives.
bool CartPanel::isDriving(const std::vector<Proc> & procs)
{
  static const QRegularExpression driver(
    kPy + R"((pagard_adaptive_fixed|pose_speed_player|cbf_qp_node)\.py)");
  const Step * controller = find("cbf_qp");
  return isUp(*controller, procs) || std::any_of(
    procs.begin(), procs.end(), [](const Proc & proc) {return matches(driver, proc.cmd);});
}

// For the confirm dialog of a step that follows the route: the map the
// localization is on and whether the route lies on it. Reading a map takes a
// moment, so the answer is kept until one of the two files changes.
QString CartPanel::routeNote(const std::vector<Proc> & procs, bool * off_map)
{
  *off_map = false;
  const QString route = routeToFollow();
  const QString route_name = route.isEmpty() ? "ของ nav_path.py" : QFileInfo(route).fileName();
  QString map;
  if (!runningMap(procs, &map)) {
    // No map to check it against; the localization is already listed as not started.
    return route.isEmpty() ? QString() : "เส้นทาง: " + route_name;
  }
  if (map.isEmpty()) {
    return "แผนที่: ตาม localization.yaml (Localization ไม่ได้เปิดจากปุ่มนี้)\n"
           "เช็กเองว่าเส้นทาง " + route_name + " เป็นของแผนที่นั้น";
  }
  const Map * known = mapAt(map);
  const QString map_name = known ? known->label : QFileInfo(map).fileName();
  const QFileInfo map_file(map);
  const QFileInfo route_file(route);
  const QString checked = QString("%1|%2|%3|%4|%5").arg(map).arg(map_file.size()).arg(
    map_file.lastModified().toMSecsSinceEpoch()).arg(route).arg(
    route_file.lastModified().toMSecsSinceEpoch());
  if (checked != route_checked_) {
    QApplication::setOverrideCursor(Qt::WaitCursor);
    route_share_ = route.isEmpty() ? -1.0 : routeOnMap(map, route);
    QApplication::restoreOverrideCursor();
    route_checked_ = checked;
  }
  if (route_share_ < 0.0) {
    return "แผนที่: " + map_name + "\nตรวจเส้นทาง " + route_name +
           " กับแผนที่ไม่ได้ เช็กเองว่าเป็นของแผนที่นี้";
  }
  if (route_share_ < kRouteOnMapMin) {
    *off_map = true;
    return QString(
      "⚠ แผนที่ที่ใช้อยู่: %1\n"
      "เส้นทาง %2 อยู่บนแผนที่นี้แค่ %3% น่าจะเป็นเส้นทางของแผนที่อื่น\n"
      "ถ้าเริ่ม รถจะวิ่งไปยังจุดที่ไม่ใช่ถนน\n"
      "เลือกเส้นทางของแผนที่นี้ได้ที่ปุ่ม \"เส้นทาง\" ใต้ขั้นนี้").arg(map_name, route_name).arg(
      qRound(route_share_ * 100));
  }
  return "แผนที่: " + map_name + " · เส้นทาง " + route_name + " อยู่บนแผนที่นี้";
}

void CartPanel::buildUi()
{
  setStyleSheet(panelStyle(palette().color(QPalette::Window).lightness() < 128));

  auto * title = new QLabel("ระบบรถกอล์ฟ");
  title->setObjectName("title");
  running_chip_ = new QLabel;
  busy_chip_ = new QLabel;
  ended_chip_ = new QLabel;
  auto * header = new QHBoxLayout;
  header->setSpacing(6);
  header->addWidget(title);
  header->addStretch();
  for (QLabel * chip : {busy_chip_, ended_chip_, running_chip_}) {
    chip->setObjectName("chip");
    header->addWidget(chip);
  }
  restyle(busy_chip_, "tone", "busy");
  restyle(ended_chip_, "tone", "ended");

  // One segment per step, in start-up order: the whole chain at a glance.
  auto * strip = new QHBoxLayout;
  strip->setSpacing(3);
  for (Step & step : steps_) {
    if (step.tool) {
      continue;
    }
    step.segment = new QFrame;
    step.segment->setObjectName("segment");
    step.segment->setFixedHeight(4);
    strip->addWidget(step.segment);
  }

  auto * layout = new QVBoxLayout;
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(4);
  layout->addLayout(header);
  layout->addLayout(strip);
  layout->addSpacing(2);
  for (size_t i = 0; i < steps_.size(); ++i) {
    if (!steps_[i].tool) {
      layout->addWidget(buildCard(steps_[i], i));
    }
  }
  auto * tools = new QLabel("เครื่องมือ");
  tools->setObjectName("section");
  layout->addSpacing(4);
  layout->addWidget(tools);
  for (size_t i = 0; i < steps_.size(); ++i) {
    if (steps_[i].tool) {
      layout->addWidget(buildCard(steps_[i], i));
    }
  }
  // Shown once RViz has displays with these names; the Displays panel still
  // has the full tree.
  layer_box_ = new QWidget;
  auto * layer_grid = new QGridLayout(layer_box_);
  layer_grid->setContentsMargins(0, 6, 0, 0);
  layer_grid->setHorizontalSpacing(6);
  layer_grid->setVerticalSpacing(6);
  auto * section = new QLabel("แสดงผล");
  section->setObjectName("section");
  layer_grid->addWidget(section, 0, 0, 1, 3);
  for (size_t i = 0; i < layers_.size(); ++i) {
    Layer & layer = layers_[i];
    layer.chip = new QPushButton(layer.label);
    layer.chip->setObjectName("layer");
    layer.chip->setCheckable(true);
    layer.chip->setCursor(Qt::PointingHandCursor);
    layer.chip->setToolTip(layer.displays.join(", "));
    connect(layer.chip, &QPushButton::clicked, this, [this, i](bool on) {setLayer(i, on);});
    layer_grid->addWidget(layer.chip, 1 + static_cast<int>(i) / 3, static_cast<int>(i) % 3);
  }
  layer_box_->setVisible(false);
  layout->addWidget(layer_box_);

  auto * hint = new QLabel("หยุด = Ctrl+C ในหน้าต่างนั้น · ปิด RViz แล้วทุกขั้นยังรันต่อ");
  hint->setObjectName("hint");
  hint->setWordWrap(true);
  layout->addWidget(hint);
  layout->addStretch();

  // The column scrolls when the dock is shorter than it. RViz inside the cart
  // launcher is 939 px tall (2026-10-21) and the column about 900, so without
  // this the last card ran off the bottom and its button could not be pressed.
  column_ = new QWidget;
  column_->setLayout(layout);
  scroll_ = new QScrollArea;
  scroll_->setFrameShape(QFrame::NoFrame);
  scroll_->setWidgetResizable(true);
  scroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll_->setWidget(column_);
  column_->setAutoFillBackground(false);  // setWidget() turned it on
  column_->installEventFilter(this);
  fitColumnWidth();
  auto * outer = new QVBoxLayout;
  outer->setContentsMargins(0, 0, 0, 0);
  outer->addWidget(scroll_);
  setLayout(outer);
}

// The dock is as wide as the widest line of the column, scroll bar included,
// as it was before the column scrolled: status lines are one line, so the dock
// widens rather than clips them.
void CartPanel::fitColumnWidth()
{
  scroll_->setMinimumWidth(
    column_->minimumSizeHint().width() + scroll_->verticalScrollBar()->sizeHint().width());
}

QFrame * CartPanel::buildCard(Step & step, size_t index)
{
  step.card = new QFrame;
  step.card->setObjectName("card");
  QString tip = step.command;
  if (!step.records.isEmpty()) {
    tip += "\nเก็บลง " + tilde(step.records);
  }
  if (!step.bags.isEmpty()) {
    tip += "\nเก็บลง " + tilde(step.bags) + "/ หนึ่งโฟลเดอร์ต่อการกดเปิดหนึ่งครั้ง";
  }
  step.card->setToolTip(tip);

  step.badge = new QLabel(step.tool ? QStringLiteral("●") : QString::number(step.number));
  step.badge->setObjectName("badge");
  step.badge->setAlignment(Qt::AlignCenter);
  step.badge->setFixedSize(24, 24);

  auto * name = new QLabel(step.name);
  name->setObjectName("name");
  step.status = new QLabel;  // one line, so the dock widens rather than clips it
  step.status->setObjectName("status");
  auto * texts = new QVBoxLayout;
  texts->setSpacing(0);
  texts->addWidget(name);
  texts->addWidget(step.status);

  step.button = new QPushButton;
  step.button->setObjectName("action");
  step.button->setCursor(Qt::PointingHandCursor);
  connect(step.button, &QPushButton::clicked, this, [this, index]() {onClicked(index);});

  auto * row = new QHBoxLayout;
  row->setSpacing(10);
  row->addWidget(step.badge);
  row->addLayout(texts, 1);
  row->addWidget(step.button);
  auto * column = new QVBoxLayout(step.card);
  column->setContentsMargins(8, 4, 8, 4);  // the tool buttons fit a 939 px RViz unscrolled
  column->setSpacing(4);
  column->addLayout(row);
  if (!step.records.isEmpty() || !step.logs.isEmpty() || !step.bags.isEmpty()) {
    // In line with the step's name, and selectable so the path can be copied.
    step.where = new QLabel;
    step.where->setObjectName("where");
    step.where->setContentsMargins(34, 0, 0, 2);
    step.where->setTextInteractionFlags(Qt::TextSelectableByMouse);
    column->addWidget(step.where);
  }
  if (step.key == "localization" && !maps_.empty()) {
    column->addWidget(buildMapRow());
  }
  if (step.follows_route) {
    column->addWidget(buildRouteRow());
    column->addWidget(buildKnobRows());
  }
  return step.card;
}

// One chip per map, under the step that loads it. The lit one is the map the
// localization runs on, or will be started on.
QWidget * CartPanel::buildMapRow()
{
  auto * box = new QWidget;
  auto * grid = new QGridLayout(box);
  grid->setContentsMargins(34, 0, 0, 2);  // in line with the step's name
  grid->setHorizontalSpacing(6);
  grid->setVerticalSpacing(4);
  auto * label = new QLabel("แผนที่");
  label->setObjectName("section");
  grid->addWidget(label, 0, 0);
  for (size_t i = 0; i < maps_.size(); ++i) {
    Map & map = maps_[i];
    map.chip = new QPushButton(map.label);
    map.chip->setObjectName("map");
    map.chip->setCheckable(true);
    map.chip->setCursor(Qt::PointingHandCursor);
    connect(map.chip, &QPushButton::clicked, this, [this, i]() {onMapClicked(i);});
    grid->addWidget(map.chip, static_cast<int>(i) / 3, 1 + static_cast<int>(i) % 3);
  }
  grid->setColumnStretch(4, 1);
  return box;
}

// One chip per route, with its length beside it, under the step that follows
// it. The lit one is the route the controller runs on, or will be started on.
// The chips come and go with the files, so refreshRoutes() makes them.
QWidget * CartPanel::buildRouteRow()
{
  route_box_ = new QWidget;
  route_grid_ = new QGridLayout(route_box_);
  route_grid_->setContentsMargins(34, 0, 0, 2);  // in line with the step's name
  route_grid_->setHorizontalSpacing(6);
  route_grid_->setVerticalSpacing(4);
  auto * label = new QLabel("เส้นทาง");
  label->setObjectName("section");
  route_grid_->addWidget(label, 0, 0);
  route_grid_->setColumnStretch(3, 1);
  return route_box_;
}

// The solver's switches under the step that starts it: the constraints in a
// row, each over the number it is tuned by, then a line on where a change goes
// and a way back to the preset. Three rows, as the dock has little height left.
QWidget * CartPanel::buildKnobRows()
{
  auto * box = new QWidget;
  auto * grid = new QGridLayout(box);
  grid->setContentsMargins(34, 4, 0, 2);  // in line with the step's name
  grid->setHorizontalSpacing(4);
  grid->setVerticalSpacing(4);
  for (size_t i = 0; i < knobs_.size(); ++i) {
    Knob & knob = knobs_[i];
    const int column = static_cast<int>(i / 2);
    if (knob.flag) {
      knob.chip = new QPushButton(knob.label);
      knob.chip->setObjectName("knob");
      knob.chip->setCheckable(true);
      knob.chip->setCursor(Qt::PointingHandCursor);
      knob.chip->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
      connect(knob.chip, &QPushButton::clicked, this, [this, i](bool on) {onKnobChanged(i, on);});
      grid->addWidget(knob.chip, 0, column);
      continue;
    }
    knob.box = new QDoubleSpinBox;
    knob.box->setObjectName("knob");
    knob.box->setRange(knob.low, knob.high);
    knob.box->setSingleStep(knob.step);
    knob.box->setDecimals(knob.decimals);
    knob.box->setPrefix(knob.label);
    knob.box->setSuffix(knob.unit);
    knob.box->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    // A value goes out on Enter, on leaving the box or on an arrow, not per key.
    knob.box->setKeyboardTracking(false);
    knob.box->setFocusPolicy(Qt::StrongFocus);
    knob.box->installEventFilter(this);
    connect(
      knob.box, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
      [this, i](double value) {onKnobChanged(i, value);});
    grid->addWidget(knob.box, 1, column);
  }
  knob_note_ = new QLabel;
  knob_note_->setObjectName("where");
  knob_note_->setWordWrap(true);
  auto * reset = new QPushButton("ค่าเริ่มต้น");
  reset->setObjectName("knob");
  reset->setCursor(Qt::PointingHandCursor);
  reset->setToolTip(
    "กลับไปใช้ค่าใน src/nav_path_launch/config/cbf_qp_preset.yaml\n"
    "(ถ้า CBF-QP รันอยู่ ค่าในตัวที่รันอยู่เปลี่ยนตามทันที)");
  connect(reset, &QPushButton::clicked, this, &CartPanel::resetKnobs);
  grid->addWidget(knob_note_, 2, 0, 1, 2);
  grid->addWidget(reset, 2, 2);
  return box;
}

CartPanel::Step * CartPanel::find(const QString & key)
{
  for (Step & step : steps_) {
    if (step.key == key) {
      return &step;
    }
  }
  return nullptr;
}

std::vector<CartPanel::Proc> CartPanel::listProcs()
{
  std::vector<Proc> procs;
  const QStringList entries =
    QDir(QStringLiteral("/proc")).entryList(QDir::Dirs | QDir::NoDotAndDotDot);
  for (const QString & name : entries) {
    bool ok = false;
    const pid_t pid = name.toInt(&ok);
    if (!ok) {
      continue;
    }
    QFile file("/proc/" + name + "/cmdline");
    if (!file.open(QIODevice::ReadOnly)) {
      continue;
    }
    QByteArray raw = file.readAll();
    const pid_t pgid = getpgid(pid);
    if (raw.isEmpty() || pgid < 0) {
      continue;  // kernel thread, or gone meanwhile
    }
    if (raw.endsWith('\0')) {
      raw.chop(1);
    }
    raw.replace('\0', ' ');
    procs.push_back({pid, pgid, QString::fromLocal8Bit(raw)});
  }
  return procs;
}

bool CartPanel::isUp(const Step & step, const std::vector<Proc> & procs) const
{
  return std::any_of(
    procs.begin(), procs.end(), [&step](const Proc & proc) {
      return matches(step.running, proc.cmd) || matches(step.starting, proc.cmd);
    });
}

// Signal the whole process group, which is exactly what Ctrl+C in its terminal
// does, only when the group belongs to this step: led by this step's terminal
// script, or by the step itself when it was started by hand. The logger inside
// step 6's group, for one, gets only its own signal.
bool CartPanel::ownsGroup(
  const Step & step, const Proc & proc, const std::vector<Proc> & procs) const
{
  if (proc.pgid <= 1 || proc.pgid == getpgrp()) {
    return false;
  }
  for (const Proc & leader : procs) {
    if (leader.pid == proc.pgid) {
      return matches(step.terminal, leader.cmd) || matches(step.running, leader.cmd) ||
             matches(step.starting, leader.cmd);
    }
  }
  return false;
}

// The other step whose process group this one runs in, if any.
const CartPanel::Step * CartPanel::ownerOf(
  const Step & step, const Proc & proc, const std::vector<Proc> & procs) const
{
  for (const Proc & leader : procs) {
    if (leader.pid != proc.pgid) {
      continue;
    }
    for (const Step & other : steps_) {
      if (&other != &step &&
        (matches(other.terminal, leader.cmd) || matches(other.running, leader.cmd) ||
        matches(other.starting, leader.cmd)))
      {
        return &other;
      }
    }
  }
  return nullptr;
}

void CartPanel::onClicked(size_t index)
{
  Step & step = steps_[index];
  const bool stopping = step.stop_level > 0 || isUp(step, listProcs());

  if (stopping) {
    if (step.stop_level == 0 && !step.confirm_stop.isEmpty() &&
      QMessageBox::warning(
        this, step.label, step.confirm_stop, QMessageBox::Yes | QMessageBox::Cancel,
        QMessageBox::Cancel) != QMessageBox::Yes)
    {
      return;
    }
    stop(step, listProcs());
  } else {
    if (!step.device.isEmpty() && !QFile::exists(step.device) &&
      QMessageBox::warning(
        this, step.label,
        "ไม่พบ " + step.device + "\n"
        "เครื่องยังไม่เห็นอุปกรณ์ เช็กสาย USB ก่อน\n\n"
        "เปิดต่อไหม? (launch จะรออุปกรณ์อีก 10 วินาที)",
        QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Yes)
    {
      return;
    }
    if (step.follows_route && !route_.isEmpty()) {
      if (!QFile::exists(route_)) {
        QMessageBox::warning(
          this, step.label,
          "ไม่พบไฟล์เส้นทาง\n" + route_ + "\n\nเลือกเส้นทางอื่นที่ปุ่ม \"เส้นทาง\" ใต้ขั้นนี้");
        return;
      }
      // nav_path.py reads its file again whenever it changes: the cart would
      // be steered along a path that is still being written.
      const std::vector<Proc> procs = listProcs();
      for (const Step & other : steps_) {
        if (other.records == route_ && isUp(other, procs)) {
          QMessageBox::warning(
            this, step.label,
            "\"" + other.label + "\" กำลังเก็บจุดลงไฟล์เส้นทางนี้อยู่\n" + tilde(route_) + "\n\n"
            "หยุดเก็บ path ก่อน แล้วค่อยเปิดขั้นนี้");
          return;
        }
      }
    }
    if (!step.confirm_start.isEmpty()) {
      const std::vector<Proc> procs = listProcs();
      QStringList missing;
      for (const QString & key : step.needs) {
        const Step * need = find(key);
        if (need && !isUp(*need, procs)) {
          missing << need->label;
        }
      }
      QString text = step.confirm_start;
      if (!missing.isEmpty()) {
        text += "\n\n⚠ ยังไม่ได้เปิด: " + missing.join(", ");
      }
      bool off_map = false;
      if (step.follows_route) {
        QStringList notes;
        const QString note = routeNote(procs, &off_map);
        if (!note.isEmpty()) {
          notes << note;
        }
        notes << equationNote();
        // A recorded route carries the speed it was driven at.
        const QString route = routeToFollow();
        if (!route.isEmpty() && sheet(route).speed) {
          notes << QString("ความเร็วในเส้นทาง: สูงสุด %1 m/s").arg(mps(sheet(route).v_max));
        }
        if (!notes.isEmpty()) {
          text += "\n\n" + notes.join("\n");
        }
      }
      const QMessageBox::StandardButton answer = off_map ?
        QMessageBox::warning(
        this, step.label, text, QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel) :
        QMessageBox::question(
        this, step.label, text, QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel);
      if (answer != QMessageBox::Yes) {
        return;
      }
    }
    if (!step.records.isEmpty() && !confirmRecording(step, listProcs())) {
      return;
    }
    const qint64 free = step.bags.isEmpty() ? -1 : freeBytes(step.bags);
    if (free >= 0 && free < kBagLowBytes &&
      QMessageBox::warning(
        this, step.label,
        "ดิสก์เหลือ " + bytesText(free) + "\n"
        "bag ตอนรถวิ่งใช้ราว 8 GB ต่อชั่วโมง ถ้าดิสก์เต็ม bag จะหยุดเก็บกลางทาง\n\n"
        "เริ่มเก็บต่อไหม?",
        QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Yes)
    {
      return;
    }
    if (step.key == "localization" && !map_.isEmpty() && !QFile::exists(map_)) {
      QMessageBox::warning(
        this, step.label, "ไม่พบไฟล์แผนที่\n" + map_ + "\n\nเลือกแผนที่อื่น หรือเปิด RViz ใหม่");
      return;
    }
    start(step, listProcs());
  }
  refresh();
}

// A recorder empties its file when it starts. So a recording worth keeping is
// moved aside first, under the time it was made, and the dialog says so; it
// also says when nothing will be recorded yet, and when the speed will not be
// a measured one. With none of these, no dialog.
bool CartPanel::confirmRecording(const Step & step, const std::vector<Proc> & procs)
{
  QString followed;
  if (runningRoute(procs, &followed) && followed == step.records) {
    QMessageBox::warning(
      this, step.label,
      "CBF-QP กำลังวิ่งตามไฟล์นี้อยู่\n" + tilde(step.records) + "\n\n"
      "get_path.py จะล้างไฟล์แล้วเขียนใหม่ เส้นทางของรถจะหายไปกลางทาง\n"
      "หยุด CBF-QP ก่อน แล้วค่อยเก็บ path ใหม่");
    return false;
  }
  const Recorded old = recorded(step.records);
  const bool keep = old.rows >= kKeepRows;
  QStringList missing;
  for (const QString & key : step.needs) {
    const Step * need = find(key);
    if (need && !isUp(*need, procs)) {
      missing << need->label;
    }
  }
  const Step * meter = find(step.speed_from);
  const bool unmeasured = meter && !isUp(*meter, procs);
  if (!keep && missing.isEmpty() && !unmeasured) {
    return true;
  }
  const QFileInfo file(step.records);
  QString text;
  QString kept;
  if (keep) {
    const QString stem = file.path() + "/" + file.completeBaseName() + "_" +
      file.lastModified().toString("yyyyMMdd_HHmmss");
    kept = stem + "." + file.suffix();
    for (int i = 2; QFile::exists(kept); ++i) {
      kept = QString("%1_%2.%3").arg(stem).arg(i).arg(file.suffix());
    }
    text = "มี path ที่เก็บไว้แล้ว " + describe(old) + " (" +
      file.lastModified().toString("d/M/yyyy HH:mm") + ")\n"
      "get_path.py เริ่มไฟล์ใหม่ทุกครั้ง ของเดิมจะถูกย้ายไปเก็บเป็น\n" + tilde(kept) + "\n\n";
    if (route_ == step.records) {
      text += "เส้นทางที่เลือกไว้ให้ CBF-QP จะยังเป็นของเดิมนี้ (ไฟล์ที่ย้ายไป)\n\n";
    }
  }
  if (!missing.isEmpty()) {
    text += "⚠ ยังไม่ได้เปิด: " + missing.join(", ") +
      "\nจะยังไม่มีจุดถูกเก็บจนกว่าจะมีตำแหน่งรถ\n\n";
  }
  if (unmeasured) {
    text += "⚠ ยังไม่ได้เปิด: " + meter->label +
      "\nความเร็วที่เก็บจะคิดจากระยะที่รถขยับ แทนค่าที่วัดได้ ซึ่งแกว่งกว่า\n\n";
  }
  text += "เริ่มเก็บ path ใหม่ลง " + tilde(step.records) + " เลยไหม?";
  if (QMessageBox::question(
      this, step.label, text, QMessageBox::Yes | QMessageBox::Cancel,
      QMessageBox::Cancel) != QMessageBox::Yes)
  {
    return false;
  }
  if (keep && !QFile::rename(step.records, kept)) {
    QMessageBox::warning(
      this, step.label, "ย้ายไฟล์เดิมไม่ได้ จึงยังไม่เริ่มเก็บ\n" + tilde(step.records));
    return false;
  }
  if (keep && route_ == step.records) {
    // The choice was this path, not whatever is recorded next under its name.
    selectRoute(kept);
  }
  return true;
}

// With the controller down this only picks the route for its next start. With
// it up nothing changes: the cart is following that route, so the controller
// is stopped first, by hand.
void CartPanel::onRouteClicked(const QString & path)
{
  QString running;
  const bool up = runningRoute(listProcs(), &running);
  if (!QFile::exists(path)) {
    QMessageBox::warning(this, "เส้นทาง", "ไม่พบไฟล์เส้นทาง\n" + path);
  } else if (!up) {
    selectRoute(path);
  } else if (running != path) {
    QMessageBox::warning(
      this, "เส้นทาง",
      "CBF-QP กำลังวิ่งตามเส้นทาง " + QFileInfo(running).fileName() + " อยู่\n"
      "เปลี่ยนเส้นทางตอนรถวิ่งไม่ได้\n\n"
      "หยุด CBF-QP ก่อน เลือกเส้นทาง แล้วค่อยเปิดใหม่");
  }
  refresh();
}

// With the localization down this only picks the map for its next start. With
// it up, a map is read once at start-up, so changing it means a restart: asked
// first, and refused while something is driving the cart by the pose.
void CartPanel::onMapClicked(size_t index)
{
  const Map & map = maps_[index];
  Step & step = *find("localization");
  const std::vector<Proc> procs = listProcs();
  QString running;
  const bool up = runningMap(procs, &running);
  if (!QFile::exists(map.path)) {
    QMessageBox::warning(
      this, "แผนที่", "ไม่พบไฟล์แผนที่\n" + map.path + "\n\nเปิด RViz ใหม่เพื่ออ่านรายการแผนที่อีกครั้ง");
  } else if (!up) {
    selectMap(map.path);
  } else if (running != map.path) {
    if (isDriving(procs)) {
      QMessageBox::warning(
        this, "แผนที่",
        "ตัวควบคุมรถ (CBF-QP / pagard) กำลังรันอยู่\n"
        "เปลี่ยนแผนที่ตอนนี้ตำแหน่งรถจะกระโดดไปอยู่ในแผนที่ใหม่ขณะที่รถยังวิ่งตามเส้นทางเดิม\n\n"
        "หยุดตัวควบคุมก่อน แล้วค่อยเปลี่ยนแผนที่");
    } else if (QMessageBox::question(
        this, "แผนที่",
        "เปลี่ยนแผนที่เป็น " + map.label + "?\n\n"
        "Localization จะหยุดแล้วเปิดใหม่ด้วยแผนที่นี้\n"
        "ตำแหน่งรถจะเริ่มที่จุดเริ่มต้นของแผนที่ใหม่ ถ้ารถไม่ได้อยู่ตรงนั้นให้ตั้งด้วย 2D Pose Estimate\n"
        "เส้นทางของแผนที่เดิมใช้กับแผนที่นี้ไม่ได้",
        QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel) == QMessageBox::Yes)
    {
      selectMap(map.path);
      step.restart = true;
      stop(step, listProcs());
    }
  }
  refresh();
}

void CartPanel::start(Step & step, const std::vector<Proc> & procs)
{
  // A terminal left from a run that ended by itself only waits for Enter;
  // close it so the new run does not open next to it.
  for (const Proc & proc : procs) {
    if (matches(step.terminal, proc.cmd)) {
      ::kill(proc.pid, SIGHUP);
    }
  }
  const QString script = writeScript(step);
  if (script.isEmpty()) {
    QMessageBox::warning(this, step.label, "เขียนสคริปต์ไม่ได้: " + script_dir_);
    return;
  }
  if (!openTerminal(step, script)) {
    QMessageBox::warning(
      this, step.label, "เปิด terminal ไม่ได้ (ไม่พบ terminator หรือ gnome-terminal)");
    return;
  }
  step.start_clock.start();
}

void CartPanel::stop(Step & step, const std::vector<Proc> & procs)
{
  // Ctrl+C first, as in the terminal; TERM, then KILL, if pressed again.
  static const int kSignals[] = {SIGINT, SIGTERM, SIGKILL};
  const int sig = kSignals[std::min(step.stop_level, 2)];
  std::set<pid_t> groups;
  for (const Proc & proc : procs) {
    if (!matches(step.running, proc.cmd) && !matches(step.starting, proc.cmd)) {
      continue;
    }
    if (ownsGroup(step, proc, procs)) {
      if (groups.insert(proc.pgid).second) {
        ::kill(-proc.pgid, sig);
      }
    } else {
      ::kill(proc.pid, sig);
    }
  }
  step.stop_level = std::min(step.stop_level + 1, 3);
  step.stop_clock.start();
  step.start_clock.invalidate();
}

// The command goes in a script because terminator joins everything after -x
// into one string, which breaks any quoting; the terminal only gets
// "bash <script>". QSaveFile renames over the old copy, so a terminal still
// reading it keeps the old file.
QString CartPanel::writeScript(const Step & step) const
{
  if (!QDir().mkpath(script_dir_)) {
    return QString();
  }
  const QString path = script_dir_ + "/" + step.key + ".sh";
  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    return QString();
  }
  const QString text =
    "# เขียนโดยปุ่ม \"" + step.label + "\" ใน RViz (cart_rviz_panel) ทุกครั้งที่กดเปิด\n"
    "cd " + kWorkspace + " && source install/setup.bash && " + commandOf(step) + "\n"
    "code=$?\n"
    "echo\n"
    "echo \"== " + step.label + " จบแล้ว (exit $code) — กด Enter เพื่อปิดหน้าต่างนี้\"\n"
    "read -r\n";
  file.write(text.toUtf8());
  return file.commit() ? path : QString();
}

bool CartPanel::openTerminal(const Step & step, const QString & script) const
{
  QProcess process;
  if (!QStandardPaths::findExecutable("terminator").isEmpty()) {
    process.setProgram("terminator");
    process.setArguments({"-T", step.label, "-x", "bash", script});
  } else if (!QStandardPaths::findExecutable("gnome-terminal").isEmpty()) {
    process.setProgram("gnome-terminal");
    process.setArguments({"--title", step.label, "--", "bash", script});
  } else {
    process.setProgram("x-terminal-emulator");
    process.setArguments({"-T", step.label, "-e", "bash", script});
  }
  // The step itself talks to the terminal's own tty. Without this the
  // terminal program would keep RViz's stdout and stderr open (a pipe, when
  // RViz runs under ros2 launch) for as long as any window stays open.
  process.setStandardInputFile(QProcess::nullDevice());
  process.setStandardOutputFile(QProcess::nullDevice());
  process.setStandardErrorFile(QProcess::nullDevice());
  return process.startDetached();
}

void CartPanel::refresh()
{
  const std::vector<Proc> procs = listProcs();
  int chain_count = 0;    // the numbered steps; the tools are not counted as running
  int running_count = 0;
  int busy_count = 0;
  int ended_count = 0;
  for (Step & step : steps_) {
    const Proc * running = nullptr;
    const Proc * preparing = nullptr;
    bool terminal_open = false;
    for (const Proc & proc : procs) {
      if (!running && matches(step.running, proc.cmd)) {
        running = &proc;
      }
      if (!preparing && matches(step.starting, proc.cmd)) {
        preparing = &proc;
      }
      terminal_open = terminal_open || matches(step.terminal, proc.cmd);
    }
    const bool up = running || preparing;

    if (up || (step.start_clock.isValid() && step.start_clock.elapsed() > kStartWaitMs)) {
      step.start_clock.invalidate();
    }
    if (!up && step.stop_level > 0) {
      // Stopped from here: close its terminal as well, like closing the window
      // after Ctrl+C. One that ended by itself stays open so it can be read.
      for (const Proc & proc : procs) {
        if (matches(step.terminal, proc.cmd)) {
          ::kill(proc.pid, SIGHUP);
        }
      }
      step.stop_level = 0;
      terminal_open = false;
    }
    if (step.restart && !up && step.stop_level == 0) {
      step.restart = false;
      if (step.key != "localization" || map_.isEmpty() || QFile::exists(map_)) {
        start(step, procs);
      }
    }

    QString state;   // run, busy, ended or off
    QString text;
    QString tip;
    QString action;
    QString kind;    // start, stop or force; empty leaves the button disabled
    if (step.stop_level > 0) {
      state = "busy";
      const bool can_force = step.stop_clock.elapsed() >= kStopWaitMs;
      text = can_force ? "ยังไม่หยุด · กดบังคับหยุดได้" :
        step.restart ? "กำลังเปลี่ยนแผนที่…" : "กำลังหยุด…";
      tip = "ส่ง Ctrl+C ไปแล้ว ถ้าผ่านไป 5 วินาทียังไม่หยุด กดอีกครั้งเพื่อบังคับหยุด";
      action = !can_force ? "รอ…" : step.stop_level == 1 ? "บังคับหยุด" : "ฆ่าทิ้ง";
      kind = can_force ? "force" : "";
    } else if (running) {
      state = "run";
      const Step * owner = ownerOf(step, *running, procs);
      if (owner) {
        text = QString("รันอยู่ · มากับขั้นที่ %1").arg(owner->number);
        tip = "รันอยู่ในกลุ่มของ " + owner->label + " — หยุดขั้นนั้นแล้วตัวนี้หยุดด้วย";
      } else {
        const qint64 seconds = secondsRunning(running->pid);
        text = seconds < 0 ? "รันอยู่" : "รันอยู่ · " + formatUptime(seconds);
      }
      action = "■ หยุด";
      kind = "stop";
    } else if (preparing) {
      state = "busy";
      text = "กำลังเตรียม · ดู terminal";
      for (const Proc & proc : procs) {
        for (const Wait & wait : step.waits) {
          if (proc.pgid == preparing->pgid && matches(wait.process, proc.cmd)) {
            text = wait.text;
          }
        }
      }
      tip = "start_cart.sh กำลังรีเซ็ต can0 และรอ STM32 ก่อนเปิด low_level";
      action = "■ หยุด";
      kind = "stop";
    } else if (step.start_clock.isValid()) {
      state = "busy";
      text = "กำลังเปิด…";
      action = "รอ…";
    } else if (terminal_open) {
      state = "ended";
      text = "จบเอง · ดูสาเหตุใน terminal";
      tip = "หน้าต่าง terminal ของขั้นนี้ยังเปิดค้างไว้ให้อ่านข้อความก่อนจบ";
      action = "↻ เปิดใหม่";
      kind = "start";
    } else {
      state = "off";
      text = "ปิดอยู่";
      action = "▶ เปิด";
      kind = "start";
    }
    if ((state == "off" || state == "ended") && !step.device.isEmpty() &&
      !QFile::exists(step.device))
    {
      state = "busy";
      text = "ไม่พบ " + step.device + " · เช็กสาย USB";
      tip = "เครื่องยังไม่เห็นอุปกรณ์ (ดูได้จาก lsusb) เสียบสาย USB ให้แน่น "
        "หรือลองสายหรือพอร์ตอื่น แล้วรอให้ " + step.device + " ขึ้นก่อนกดเปิด";
    }
    QString note;    // under where its CSV is: the speed that goes into it
    if (!step.records.isEmpty()) {
      // What it has recorded says more than how long it has been running, and
      // an empty file means no pose is reaching it.
      const Recorded done = recorded(step.records);
      if (state == "off") {
        if (done.rows > 0) {
          text = "ปิดอยู่ · ไฟล์ล่าสุด " + describe(done);
        }
      } else if (state == "run") {
        if (done.rows > 0) {
          text = "เก็บแล้ว " + describe(done);
        } else {
          state = "busy";
          text = "ยังไม่มีตำแหน่งรถเข้ามา";
          tip = "get_path.py รอ /pcl_pose อยู่ เช็กว่า Localization รันและล็อกตำแหน่งแล้ว";
        }
      }
      const Step * meter = find(step.speed_from);
      if (done.rows == 0) {
        note = meter ? "พร้อมความเร็วรถจาก " + meter->label : QString();
      } else if (!done.speed) {
        note = "ไฟล์นี้ไม่มีความเร็ว (v_target)";
      } else if (!running) {
        note = "ความเร็วในไฟล์ " + speedRange(done.v_min, done.v_max);
      } else {
        // The speed of the waypoint just written: it changes as the cart is driven.
        note = "ความเร็ว " + mps(done.v_last) + " m/s · " +
          (meter && !isUp(*meter, procs) ? "คิดจากตำแหน่งรถ" : "สูงสุด " + mps(done.v_max));
      }
    }
    if (!step.bags.isEmpty()) {
      // How big the bag being recorded has grown; when off, what the last run left.
      const QString folder = bagOf(step, running);
      const Bag bag = bagAt(folder);
      if (state == "run") {
        const qint64 seconds = secondsRunning(running->pid);
        text = "อัดอยู่";
        if (seconds >= 0) {
          text += " · " + formatUptime(seconds);
        }
        if (bag.bytes > 0) {
          text += " · " + bytesText(bag.bytes);
        }
      } else if (state == "off" && !folder.isEmpty()) {
        if (bag.closed) {
          text = "ปิดอยู่ · ล่าสุด " +
            (bag.seconds >= 0 ? durationText(bag.seconds) + " · " : QString()) + bytesText(bag.bytes);
        } else {
          text = "ปิดอยู่ · bag ล่าสุดปิดไม่ครบ";
          tip = "ไม่มี metadata.yaml เพราะหยุดไม่เรียบร้อย (บังคับหยุด หรือไฟดับ)\n"
            "ซ่อมได้ด้วย: ros2 bag reindex " + tilde(folder);
        }
      }
      const qint64 free = freeBytes(step.bags);
      if (free >= 0 && free < kBagLowBytes) {
        note = "⚠ ดิสก์เหลือ " + bytesText(free) + " · bag ใช้ราว 8 GB/ชม.";
      }
    }
    chain_count += !step.tool;
    running_count += !step.tool && state == "run";
    busy_count += state == "busy";
    ended_count += state == "ended";

    refreshWhere(step, running, note);
    step.status->setText(text);
    step.status->setToolTip(tip);
    step.button->setText(action);
    step.button->setEnabled(!kind.isEmpty());
    restyle(step.button, "kind", kind);
    for (QWidget * widget :
      std::initializer_list<QWidget *>{step.card, step.badge, step.status, step.segment})
    {
      if (widget) {
        restyle(widget, "state", state);
      }
    }
    if (step.segment) {
      step.segment->setToolTip(step.label + " · " + text);
    }
  }

  running_chip_->setText(QString("รันอยู่ %1/%2").arg(running_count).arg(chain_count));
  restyle(running_chip_, "tone", running_count > 0 ? "run" : "off");
  busy_chip_->setText(QString("รอ %1").arg(busy_count));
  busy_chip_->setVisible(busy_count > 0);
  ended_chip_->setText(QString("จบเอง %1").arg(ended_count));
  ended_chip_->setVisible(ended_count > 0);
  refreshMaps(procs);
  refreshRoutes(procs);
  refreshKnobs(procs);
  refreshLayers();
}

// Where the step's CSV is. A recorder has one file. A logger starts one on
// every run: the one it has open while it runs, the last one made when off.
// `note` is a second line, about what is in the file.
void CartPanel::refreshWhere(Step & step, const Proc * running, const QString & note)
{
  if (!step.where) {
    return;
  }
  QString lead = "เก็บลง ";
  QString path = step.records;
  if (!step.bags.isEmpty()) {
    path = bagOf(step, running);
    if (path.isEmpty()) {
      path = step.bags + "/";
    } else if (!running) {
      lead = "ล่าสุด ";
    }
  }
  if (!step.logs.isEmpty()) {
    path = running ? openCsv(running->pid) : QString();
    if (path.isEmpty()) {
      // Up to the first folder named after the time: where all its runs go.
      const QString top = step.logs.left(step.logs.lastIndexOf('/', step.logs.indexOf('%')));
      const QString last = newestOf(top, step.logs.mid(top.size() + 1).split('/'));
      if (!running && !last.isEmpty()) {
        lead = "ล่าสุด ";
        path = last;
      } else {
        path = top + "/";
      }
    }
  }
  const QFileInfo file(path);
  QString shown = tilde(path);
  if (shown.size() > kWhereLineChars && !path.endsWith('/')) {
    shown = tilde(file.path()) + "/\n" + file.fileName();
  }
  step.where->setText(note.isEmpty() ? lead + shown : lead + shown + "\n" + note);
  QString tip = !file.isFile() ? path :
    QString("%1\n%L2 kB · แก้ไขล่าสุด %3").arg(path).arg((file.size() + 999) / 1000).arg(
    file.lastModified().toString("d/M/yyyy HH:mm:ss"));
  if (!step.bags.isEmpty()) {
    const qint64 free = freeBytes(step.bags);
    if (file.isDir() && !path.endsWith('/')) {
      tip += "\n" + bytesText(bagAt(path).bytes) + " · ดูข้างในด้วย ros2 bag info";
    }
    if (free >= 0) {
      tip += "\nดิสก์ว่าง " + bytesText(free);
    }
  }
  const Step * meter = find(step.speed_from);
  if (meter) {
    tip += "\n\nความเร็วรถตอนผ่านแต่ละจุดถูกเก็บลงคอลัมน์ v_target\n"
      "เป็นค่าที่ " + meter->label + " วัดได้ ถ้าขั้นนั้นปิดอยู่จะคิดจากระยะที่รถขยับแทน\n"
      "ตอนรถวิ่งตามไฟล์นี้เอง ค่านี้คือความเร็วเป้าหมายของแต่ละจุด";
    const QString routes = workspaceDir() + "/src/sim";
    if (QFileInfo(step.records).path() != routes) {
      tip += "\n\nปุ่มเส้นทางใต้ CBF-QP มีเฉพาะไฟล์ใน " + tilde(routes) +
        "\nจะให้รถวิ่งตามไฟล์นี้ ให้ย้ายไปทับไฟล์เส้นทางของ loop นั้น";
    }
  }
  step.where->setToolTip(tip);
}

// The bag of a step that records them. While it runs, the -o folder of its
// `ros2 bag record`, which a run started by hand may give relative to where it
// was started (without -o rosbag2 names it after the time, in that same
// place). When off, the newest bag in the step's folder.
QString CartPanel::bagOf(const Step & step, const Proc * running)
{
  if (!running) {
    return newestBag(step.bags);
  }
  static const QRegularExpression output(QStringLiteral(R"((?:^| )(?:-o|--output) ([^ ]+))"));
  const QString cwd = QFile::symLinkTarget(QStringLiteral("/proc/%1/cwd").arg(running->pid));
  const QRegularExpressionMatch match = output.match(running->cmd);
  if (!match.hasMatch()) {
    return newestBag(cwd);
  }
  return QDir::cleanPath(QDir(cwd).absoluteFilePath(match.captured(1)));
}

// The chips follow the files, and the lit one is the route in use. While the
// controller runs that is the route its launch was given, wherever it was
// started from, and it becomes the choice for the next start too. A chosen
// file that is gone, or is no longer a path, gives way to nav_path.py's own.
void CartPanel::refreshRoutes(const std::vector<Proc> & procs)
{
  if (!route_box_) {
    return;
  }
  Step & step = *find("cbf_qp");
  QString running;
  const bool up = runningRoute(procs, &running);
  const bool changing = step.stop_level > 0 || step.start_clock.isValid();

  std::vector<Route> found = findRoutes(running);
  const bool same = found.size() == routes_.size() && std::equal(
    found.begin(), found.end(), routes_.begin(), [](const Route & a, const Route & b) {
      return a.path == b.path && a.label == b.label;
    });
  if (!same) {
    for (Route & route : routes_) {
      // Not deleted on the spot: this may be running inside the chip's click.
      for (QWidget * widget : std::initializer_list<QWidget *>{route.chip, route.note}) {
        route_grid_->removeWidget(widget);
        widget->hide();
        widget->deleteLater();
      }
    }
    routes_ = std::move(found);
    for (size_t i = 0; i < routes_.size(); ++i) {
      Route & route = routes_[i];
      route.chip = new QPushButton(route.label);
      route.chip->setObjectName("route");
      route.chip->setCheckable(true);
      route.chip->setCursor(Qt::PointingHandCursor);
      const QString path = route.path;
      connect(route.chip, &QPushButton::clicked, this, [this, path]() {onRouteClicked(path);});
      route.note = new QLabel;
      route.note->setObjectName("where");
      route_grid_->addWidget(route.chip, static_cast<int>(i), 1);
      route_grid_->addWidget(route.note, static_cast<int>(i), 2);
    }
    route_box_->setVisible(!routes_.empty());
  }

  if (up && !changing && running != route_ && routeAt(running)) {
    selectRoute(running);
  }
  if (!routeAt(route_)) {
    const QString usual = routePath();
    if (routes_.empty()) {
      route_.clear();  // nothing to choose from: nav_path.py runs on its own default
    } else {
      selectRoute(routeAt(usual) ? usual : routes_.front().path);
    }
  }
  const bool unknown = up && !changing && !routeAt(running);
  for (Route & route : routes_) {
    const Sheet & held = sheet(route.path);
    route.chip->setChecked(!unknown && route.path == route_);
    route.chip->setEnabled(!changing);
    route.note->setText(
      held.metres >= 0.0 ? QString("%L1 ม.").arg(qRound(held.metres)) :
      QString("%L1 จุด").arg(held.rows));
    route.chip->setToolTip(
      (unknown ?
      "CBF-QP ที่รันอยู่ใช้เส้นทางที่ไม่อยู่ในรายการนี้\n" + running + "\n\n" : QString()) +
      QString("%1\n%2 · แก้ไขล่าสุด %3\n%4").arg(
        route.path, describe(held.rows, held.metres),
        QFileInfo(route.path).lastModified().toString("d/M/yyyy HH:mm"),
        held.speed ? "ความเร็วในไฟล์ " + speedRange(held.v_min, held.v_max) :
        "ไม่มีความเร็วในไฟล์ (v_target): ใช้ค่าตั้งของตัวควบคุม"));
  }
  step.card->setToolTip(commandOf(step));
}

// The lit chip is the map in use. While the localization runs that is the map
// its launch was given, wherever it was started from, and it becomes the
// choice for the next start too; none is lit when it was started by hand
// without map_path, since which map that is is written in localization.yaml.
void CartPanel::refreshMaps(const std::vector<Proc> & procs)
{
  if (maps_.empty()) {
    return;
  }
  Step & step = *find("localization");
  QString running;
  const bool up = runningMap(procs, &running);
  const bool changing = step.restart || step.stop_level > 0 || step.start_clock.isValid();
  if (up && !changing && running != map_ && mapAt(running)) {
    selectMap(running);
  }
  const bool unknown = up && !changing && !mapAt(running);
  for (Map & map : maps_) {
    const QFileInfo file(map.path);
    map.chip->setChecked(!unknown && map.path == map_);
    map.chip->setEnabled(!changing);
    map.chip->setToolTip(
      unknown ?
      "Localization ที่รันอยู่ไม่ได้เปิดด้วยแผนที่ในรายการนี้ (ใช้ map_path ใน localization.yaml)\n"
      "กดเพื่อเปิดใหม่ด้วย " + map.path :
      QString("%1\n%2 MB").arg(map.path).arg(file.size() / 1000000));
  }
  step.card->setToolTip(commandOf(step));
}

// Checked while any of the layer's displays is on, so a change made in the
// Displays panel shows here within a second.
void CartPanel::refreshLayers()
{
  rviz_common::DisplayContext * context = getDisplayContext();
  rviz_common::DisplayGroup * root = context ? context->getRootDisplayGroup() : nullptr;
  bool any = false;
  for (Layer & layer : layers_) {
    bool found = false;
    bool on = false;
    for (int i = 0; root && i < root->numDisplays(); ++i) {
      rviz_common::Display * display = root->getDisplayAt(i);
      if (layer.displays.contains(display->getName())) {
        found = true;
        on = on || display->isEnabled();
      }
    }
    layer.chip->setVisible(found);
    layer.chip->setChecked(on);
    any = any || found;
  }
  layer_box_->setVisible(any);
}

void CartPanel::setLayer(size_t index, bool on)
{
  rviz_common::DisplayContext * context = getDisplayContext();
  rviz_common::DisplayGroup * root = context ? context->getRootDisplayGroup() : nullptr;
  for (int i = 0; root && i < root->numDisplays(); ++i) {
    rviz_common::Display * display = root->getDisplayAt(i);
    if (layers_[index].displays.contains(display->getName())) {
      display->setEnabled(on);
    }
  }
}

// RViz's own node, which RViz spins on this thread, asks the solver for its
// parameters and sets them. Without RViz around the panel (no context) the
// knobs only keep what the controller's next start gets.
void CartPanel::onInitialize()
{
  rviz_common::Panel::onInitialize();
  rviz_common::DisplayContext * context = getDisplayContext();
  if (!context) {
    return;
  }
  const auto ros = context->getRosNodeAbstraction().lock();
  if (ros) {
    solver_ = std::make_shared<rclcpp::AsyncParametersClient>(ros->get_raw_node(), kSolverNode);
  }
}

// A spin box under the mouse would take the wheel and change a value of the
// running controller; only one that was clicked into does.
bool CartPanel::eventFilter(QObject * watched, QEvent * event)
{
  if (watched == column_ && event->type() == QEvent::LayoutRequest) {
    fitColumnWidth();  // its layout has already been redone for the new texts
  }
  if (event->type() == QEvent::Wheel) {
    auto * box = qobject_cast<QDoubleSpinBox *>(watched);
    if (box && !box->hasFocus()) {
      event->ignore();
      return true;
    }
  }
  return rviz_common::Panel::eventFilter(watched, event);
}

// What the knob is set to for the controller's next start: set here, or the preset.
QVariant CartPanel::storedKnob(const Knob & knob) const
{
  const QVariant saved = QSettings("cart_rviz_panel", "panel").value("solver/" + knob.name);
  if (!saved.isValid()) {
    return knob.preset;
  }
  return knob.flag ? QVariant(saved.toBool()) : QVariant(saved.toDouble());
}

// What the knob shows: the running solver's value, else the next start's.
QVariant CartPanel::knobValue(const Knob & knob) const
{
  return knob.live.isValid() ? knob.live : storedKnob(knob);
}

QVariant CartPanel::knobValue(const QString & name) const
{
  for (const Knob & knob : knobs_) {
    if (knob.name == name) {
      return knobValue(knob);
    }
  }
  return QVariant();
}

// Launch arguments for the knobs set here; the rest is left to the preset file.
QString CartPanel::knobArguments() const
{
  const QSettings settings("cart_rviz_panel", "panel");
  QString text;
  for (const Knob & knob : knobs_) {
    if (!settings.contains("solver/" + knob.name)) {
      continue;
    }
    const QVariant value = storedKnob(knob);
    text += " " + knob.name + ":=" + (
      knob.flag ? QString(value.toBool() ? "true" : "false") :
      QString::number(value.toDouble(), 'g', 6));
  }
  return text;
}

// The constraints the controller will be started with, for its confirm dialog.
QString CartPanel::equationNote() const
{
  QHash<QString, QVariant> next;
  for (const Knob & knob : knobs_) {
    next[knob.name] = storedKnob(knob);
  }
  if (next["pure_pursuit_only"].toBool()) {
    return QString("สมการ: Pure Pursuit ล้วน · L_d = %1 ม. คงที่ (ไม่ใช้ h₁ h₂)").arg(
      next["fixed_ld"].toDouble(), 0, 'f', 1);
  }
  QStringList used;
  if (next["use_h1"].toBool()) {
    used << QString("h₁ (e_max %1 ม.)").arg(next["e_max"].toDouble(), 0, 'f', 2);
  }
  if (next["use_h2"].toBool()) {
    used << QString("h₂ (a_y,max %1 m/s²)").arg(next["ay_max"].toDouble(), 0, 'f', 2);
  }
  return "สมการ: CBF-QP " + (used.isEmpty() ? QString("ไม่ใช้ h₁ h₂") : used.join(" + "));
}

// Live from the running solver when RViz can reach it, otherwise what its next
// start gets. Read again every second, so a change made in rqt_reconfigure
// shows here too.
void CartPanel::refreshKnobs(const std::vector<Proc> & procs)
{
  if (!knob_note_) {
    return;
  }
  const bool reachable = solver_ && solver_->service_is_ready();
  if (reachable && (!knob_reading_ || knob_read_clock_.elapsed() > kSolverReadWaitMs)) {
    readSolver();
  }
  if (!reachable) {
    for (Knob & knob : knobs_) {
      knob.live = QVariant();
    }
  }
  solver_live_ = reachable && std::any_of(
    knobs_.begin(), knobs_.end(), [](const Knob & knob) {return knob.live.isValid();});
  static const QRegularExpression solver(kPy + R"(cbf_qp_node\.py)");
  solver_running_ = std::any_of(
    procs.begin(), procs.end(), [](const Proc & proc) {return matches(solver, proc.cmd);});
  showKnobs();
}

void CartPanel::showKnobs()
{
  const bool pure = knobValue("pure_pursuit_only").toBool();
  for (Knob & knob : knobs_) {
    // A knob whose constraint is not in use waits, drawn dashed.
    QString idle;
    if (knob.name == "use_h1" || knob.name == "use_h2" || knob.name == "ay_max") {
      idle = pure ? "Pure Pursuit ล้วนไม่ใช้ h₁ h₂" : "";
    } else if (knob.name == "e_max") {
      idle = pure ? "Pure Pursuit ล้วนไม่ใช้ h₁ h₂" :
        !knobValue("use_h1").toBool() ? "h₁ ปิดอยู่" : "";
    } else if (knob.name == "fixed_ld") {
      idle = pure ? "" : "ใช้เฉพาะตอนเปิด Pure Pursuit";
    }
    QWidget * widget = knob.chip ? static_cast<QWidget *>(knob.chip) : knob.box;
    widget->setToolTip(idle.isEmpty() ? knob.tip : knob.tip + "\n\nตอนนี้ไม่มีผล: " + idle);
    restyle(widget, "idle", idle.isEmpty() ? "false" : "true");
    const QVariant value = knobValue(knob);
    if (knob.chip) {
      knob.chip->setChecked(value.toBool());
    } else if (!knob.box->hasFocus()) {  // not under the operator's typing
      const QSignalBlocker quiet(knob.box);
      knob.box->setValue(value.toDouble());
    }
  }
  if (!knob_error_.isEmpty() && knob_error_clock_.elapsed() > kSolverRefusalMs) {
    knob_error_.clear();
  }
  QString note;
  QString tone;
  if (!knob_error_.isEmpty()) {
    note = "⚠ ตั้งไม่ได้: " + knob_error_;
    tone = "warn";
  } else if (solver_live_) {
    note = "ปรับสดอยู่ · ครั้งหน้าใช้ค่านี้ด้วย";
  } else if (solver_running_) {
    note = "ติดต่อ solver ไม่ได้ · ใช้ครั้งหน้า";
    tone = "warn";
  } else {
    note = "ใช้ตอนเปิดครั้งถัดไป";
  }
  knob_note_->setText(note);
  restyle(knob_note_, "tone", tone);
}

void CartPanel::readSolver()
{
  std::vector<std::string> names;
  for (const Knob & knob : knobs_) {
    names.push_back(knob.name.toStdString());
  }
  knob_reading_ = true;
  knob_read_clock_.start();
  const int sets = knob_sets_;
  QPointer<CartPanel> self(this);
  solver_->get_parameters(
    names, [self, sets](std::shared_future<std::vector<rclcpp::Parameter>> future) {
      std::vector<rclcpp::Parameter> values;
      try {
        values = future.get();
      } catch (const std::exception &) {
      }
      if (self) {
        QMetaObject::invokeMethod(
          self.data(), [self, sets, values]() {
            if (self) {
              self->onSolverRead(sets, values);
            }
          }, Qt::QueuedConnection);
      }
    });
}

void CartPanel::onSolverRead(int sets, const std::vector<rclcpp::Parameter> & values)
{
  knob_reading_ = false;
  // Asked before the last change went out (it may not have been applied yet), or
  // no answer: the next second asks again.
  if (sets != knob_sets_ || values.size() != knobs_.size()) {
    return;
  }
  QSettings settings("cart_rviz_panel", "panel");
  for (size_t i = 0; i < knobs_.size(); ++i) {
    Knob & knob = knobs_[i];
    const rclcpp::Parameter & value = values[i];
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      knob.live = value.as_bool();
    } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      knob.live = value.as_double();
    } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      knob.live = static_cast<double>(value.as_int());
    } else {
      knob.live = QVariant();  // a solver from before this switch
      continue;
    }
    // Changed elsewhere (rqt_reconfigure, ros2 param set): what runs is kept for
    // the next start too, as a map or a route the steps were started on is.
    if (knob.live != storedKnob(knob)) {
      settings.setValue("solver/" + knob.name, knob.live);
    }
  }
  solver_live_ = true;
  showKnobs();
}

// Kept for the next start, and sent to the running solver at once.
void CartPanel::onKnobChanged(size_t index, const QVariant & value)
{
  Knob & knob = knobs_[index];
  QSettings("cart_rviz_panel", "panel").setValue("solver/" + knob.name, value);
  knob_error_.clear();
  if (solver_live_) {
    knob.live = value;
    sendKnobs({index});
  }
  showKnobs();
}

void CartPanel::resetKnobs()
{
  QSettings settings("cart_rviz_panel", "panel");
  std::vector<size_t> all;
  for (size_t i = 0; i < knobs_.size(); ++i) {
    settings.remove("solver/" + knobs_[i].name);
    all.push_back(i);
  }
  knob_error_.clear();
  if (solver_live_) {
    for (Knob & knob : knobs_) {
      knob.live = knob.preset;
    }
    sendKnobs(all);
  }
  showKnobs();
}

void CartPanel::sendKnobs(const std::vector<size_t> & indices)
{
  std::vector<rclcpp::Parameter> parameters;
  for (const size_t i : indices) {
    const Knob & knob = knobs_[i];
    const QVariant value = knobValue(knob);
    parameters.emplace_back(
      knob.name.toStdString(),
      knob.flag ? rclcpp::ParameterValue(value.toBool()) : rclcpp::ParameterValue(value.toDouble()));
  }
  ++knob_sets_;
  QPointer<CartPanel> self(this);
  solver_->set_parameters(
    parameters,
    [self](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      QString refused;
      try {
        for (const auto & result : future.get()) {
          if (!result.successful) {
            refused = QString::fromStdString(result.reason);
          }
        }
      } catch (const std::exception & error) {
        refused = error.what();
      }
      if (self) {
        QMetaObject::invokeMethod(
          self.data(), [self, refused]() {
            if (self) {
              // A refused value comes back to what runs with the next reading.
              self->knob_error_ = refused;
              self->knob_error_clock_.start();
              self->showKnobs();
            }
          }, Qt::QueuedConnection);
      }
    });
}

void CartPanel::showEvent(QShowEvent * event)
{
  rviz_common::Panel::showEvent(event);
  if (themed_ && !tabbed_) {
    tabbed_ = true;
    // After this show, when RViz has restored its saved window layout.
    QTimer::singleShot(0, this, &CartPanel::tabWithDisplays);
  }
}

// In the cart's own RViz this panel and Displays share the left dock. As tabs,
// with this one in front, the steps get the whole column and the display tree
// is one click away.
void CartPanel::tabWithDisplays()
{
  auto * dock = qobject_cast<QDockWidget *>(parentWidget());
  auto * window = dock ? qobject_cast<QMainWindow *>(dock->parentWidget()) : nullptr;
  if (!window || dock->isFloating()) {
    return;
  }
  const Qt::DockWidgetArea area = window->dockWidgetArea(dock);
  for (QDockWidget * other : window->findChildren<QDockWidget *>()) {
    if (other != dock && other->windowTitle() == "Displays" && !other->isFloating() &&
      window->dockWidgetArea(other) == area)
    {
      window->setTabPosition(area, QTabWidget::North);
      window->tabifyDockWidget(other, dock);
      dock->raise();
      return;
    }
  }
}

}  // namespace cart_rviz_panel

PLUGINLIB_EXPORT_CLASS(cart_rviz_panel::CartPanel, rviz_common::Panel)
