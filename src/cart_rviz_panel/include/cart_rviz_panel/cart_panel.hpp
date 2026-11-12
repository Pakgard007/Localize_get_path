#ifndef CART_RVIZ_PANEL__CART_PANEL_HPP_
#define CART_RVIZ_PANEL__CART_PANEL_HPP_

#include <sys/types.h>

#include <QElapsedTimer>
#include <QHash>
#include <QRegularExpression>
#include <QString>
#include <QStringList>
#include <QVariant>

#include <memory>
#include <vector>

#include <rviz_common/panel.hpp>

class QDoubleSpinBox;
class QFrame;
class QGridLayout;
class QLabel;
class QPushButton;
class QScrollArea;
class QShowEvent;

namespace rclcpp
{
class AsyncParametersClient;
class Parameter;
}

namespace cart_rviz_panel
{

// One button per start-up step of the cart, in the order they are brought up,
// and below them the tools that are not part of that chain (recording a path).
// Each step runs in its own terminal window, like typing the command by hand,
// and its state is read from the process table every second rather than kept
// in a flag. So the panel stays right when a step is started or stopped from
// an ordinary terminal, dies by itself, or RViz is restarted, and closing
// RViz leaves everything running.
class CartPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit CartPanel(QWidget * parent = nullptr);

  // Share (0..1) of the route's points that lie on the map: within a metre or
  // two of one of its points, seen from above. Negative when either file
  // cannot be read. A route recorded on another map scores low, because every
  // map has its own origin.
  static double routeOnMap(const QString & map_path, const QString & route_path);

protected:
  void onInitialize() override;
  void showEvent(QShowEvent * event) override;
  bool eventFilter(QObject * watched, QEvent * event) override;

private:
  struct Proc
  {
    pid_t pid;
    pid_t pgid;
    QString cmd;  // /proc/<pid>/cmdline with the NULs turned into spaces
  };

  // A process in a preparing step's group that tells what it is waiting for.
  struct Wait
  {
    QRegularExpression process;
    QString text;
  };

  struct Step
  {
    QString key;                  // name of the script the terminal runs
    bool tool = false;            // not a start-up step: no number, not in the header strip
    int number = 0;
    QString name;
    QString label;                // "<number>. <name>" (a tool: its name), for dialogs and titles
    QString command;              // run from the workspace after sourcing it
    QRegularExpression running;   // a process that means the step is up
    QRegularExpression starting;  // optional: up, but not ready yet
    std::vector<Wait> waits;
    QString device;               // optional: a device node the step needs
    QString confirm_start;        // non-empty: ask before starting
    QString confirm_stop;         // non-empty: ask before stopping
    QStringList needs;            // keys listed in confirm_start when not up
    bool follows_route = false;   // confirm_start also says whether the route is on the map
    QString records;              // optional: the CSV it starts anew and adds a row to per waypoint
    QString speed_from;           // key of the step that measures the speed written to that CSV
    QString logs;                 // optional: strftime pattern of the CSV it starts on every run
    QString bags;                 // optional: the folder it records a new bag into on every run
    QRegularExpression terminal;  // the bash running this step's script

    QFrame * card = nullptr;
    QLabel * badge = nullptr;
    QLabel * status = nullptr;
    QLabel * where = nullptr;     // where its CSV is, for a step that writes one
    QPushButton * button = nullptr;
    QFrame * segment = nullptr;   // this step's part of the header strip

    QElapsedTimer start_clock;    // valid while waiting for the step to show up
    QElapsedTimer stop_clock;     // restarted on each stop signal sent from here
    int stop_level = 0;           // stop signals sent so far: 1 INT, 2 TERM, 3 KILL
    bool restart = false;         // being stopped only to be started again (new map)
  };

  // A point cloud in the workspace's src: a map the localization can run on.
  struct Map
  {
    QString path;
    QString label;
    QPushButton * chip = nullptr;
  };

  // A CSV of waypoints the controller can be started on.
  struct Route
  {
    QString path;
    QString label;
    QPushButton * chip = nullptr;
    QLabel * note = nullptr;  // how long it is
  };

  // What a CSV holds, read again only when the file has changed.
  struct Sheet
  {
    QString stamp;        // size and time of the file this was read from
    bool route = false;   // has x and y columns and enough rows to be a path
    int rows = 0;
    double metres = -1.0;
    bool speed = false;   // has a speed for its waypoints, between these two
    double v_min = 0.0;
    double v_max = 0.0;
  };

  // A switch or a value of the CBF-QP solver. Set from here it goes to the
  // running solver at once, and is kept for the controller's next start.
  struct Knob
  {
    QString name;          // parameter of /cbf_qp_solver and argument of cbf_qp.launch.py
    QString label;
    QString tip;
    bool flag = false;     // a switch (chip); otherwise a number (spin box)
    double low = 0.0;
    double high = 0.0;
    double step = 0.0;
    int decimals = 2;
    QString unit;
    QVariant preset;       // from cbf_qp_preset.yaml
    QVariant live;         // last read from the running solver; invalid when there is none
    QPushButton * chip = nullptr;
    QDoubleSpinBox * box = nullptr;
  };

  // A toggle for the RViz displays with these names (top level only).
  struct Layer
  {
    QString label;
    QStringList displays;
    QPushButton * chip = nullptr;
  };

  void addStep(
    const QString & key, const QString & name, const QString & command,
    const QString & running, const QString & starting = QString());
  Step & addTool(
    const QString & key, const QString & name, const QString & command, const QString & running);
  void findMaps();
  void readPresets();
  void buildUi();
  void fitColumnWidth();
  QFrame * buildCard(Step & step, size_t index);
  QWidget * buildMapRow();
  QWidget * buildRouteRow();
  QWidget * buildKnobRows();
  void onClicked(size_t index);
  void onMapClicked(size_t index);
  void onRouteClicked(const QString & path);
  void selectMap(const QString & path);
  void selectRoute(const QString & path);
  void start(Step & step, const std::vector<Proc> & procs);
  void stop(Step & step, const std::vector<Proc> & procs);
  void refresh();
  void refreshMaps(const std::vector<Proc> & procs);
  void refreshRoutes(const std::vector<Proc> & procs);
  void refreshWhere(Step & step, const Proc * running, const QString & note);
  static QString bagOf(const Step & step, const Proc * running);
  void refreshLayers();
  void refreshKnobs(const std::vector<Proc> & procs);
  void showKnobs();
  void readSolver();
  void onSolverRead(int sets, const std::vector<rclcpp::Parameter> & values);
  void onKnobChanged(size_t index, const QVariant & value);
  void resetKnobs();
  void sendKnobs(const std::vector<size_t> & indices);
  QVariant knobValue(const Knob & knob) const;
  QVariant knobValue(const QString & name) const;
  QVariant storedKnob(const Knob & knob) const;
  QString knobArguments() const;
  QString equationNote() const;
  void setLayer(size_t index, bool on);
  void tabWithDisplays();

  static std::vector<Proc> listProcs();
  bool isUp(const Step & step, const std::vector<Proc> & procs) const;
  bool ownsGroup(const Step & step, const Proc & proc, const std::vector<Proc> & procs) const;
  const Step * ownerOf(const Step & step, const Proc & proc, const std::vector<Proc> & procs) const;
  QString commandOf(const Step & step) const;
  bool runningMap(const std::vector<Proc> & procs, QString * path);
  const Map * mapAt(const QString & path) const;
  bool runningRoute(const std::vector<Proc> & procs, QString * path);
  std::vector<Route> findRoutes(const QString & running);
  const Route * routeAt(const QString & path) const;
  const Sheet & sheet(const QString & path);
  QString routeToFollow() const;
  bool isDriving(const std::vector<Proc> & procs);
  QString routeNote(const std::vector<Proc> & procs, bool * off_map);
  bool confirmRecording(const Step & step, const std::vector<Proc> & procs);
  QString writeScript(const Step & step) const;
  bool openTerminal(const Step & step, const QString & script) const;
  Step * find(const QString & key);

  std::vector<Step> steps_;
  std::vector<Layer> layers_;
  std::vector<Map> maps_;
  QString map_;            // the map the localization is, or will next be, started on
  std::vector<Route> routes_;
  QString route_;          // the route the controller is, or will next be, started on
  QHash<QString, Sheet> sheets_;
  QWidget * route_box_ = nullptr;
  QGridLayout * route_grid_ = nullptr;
  QString route_checked_;  // the files the share below was worked out for
  double route_share_ = -1.0;
  QString script_dir_;
  QLabel * running_chip_ = nullptr;
  QLabel * busy_chip_ = nullptr;
  QLabel * ended_chip_ = nullptr;
  QWidget * column_ = nullptr;     // every card, in scroll_
  QScrollArea * scroll_ = nullptr;
  QWidget * layer_box_ = nullptr;
  std::vector<Knob> knobs_;
  QLabel * knob_note_ = nullptr;
  QString knob_error_;     // the solver's answer to the last change it refused
  QElapsedTimer knob_error_clock_;
  std::shared_ptr<rclcpp::AsyncParametersClient> solver_;
  bool solver_live_ = false;     // its values have been read: changes go to it
  bool solver_running_ = false;  // a cbf_qp_node.py process is up
  int knob_sets_ = 0;      // changes sent; a reading asked for before the last one is stale
  bool knob_reading_ = false;
  QElapsedTimer knob_read_clock_;
  bool themed_ = false;  // the cart's own RViz: dark window, tabbed with Displays
  bool tabbed_ = false;
};

}  // namespace cart_rviz_panel

#endif  // CART_RVIZ_PANEL__CART_PANEL_HPP_
