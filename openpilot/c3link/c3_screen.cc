// C3 status screen (2026-10-18): temperature, IP, the USB link to C4, calibration, lights, the drive archive.
// No camera picture - the driver does not look at C3 while driving (only C4's HUD), and the old live view stuttered.
// Qt Widgets, no Q_OBJECT (no moc). Rotated the way openpilot does it: wl_surface_set_buffer_transform(270).
// Self-check without the panel: QT_QPA_PLATFORM=offscreen ./c3_screen --shot /tmp/shot.png
//
// Build on C3 (Qt 5.12.8, ~7 s):
//   g++ -O2 -fPIC -std=c++17 -DUSE_WAYLAND c3_screen.cc -o c3_screen \
//     $(pkg-config --cflags --libs Qt5Widgets Qt5Gui Qt5Core wayland-client) \
//     -I/usr/include/aarch64-linux-gnu/qt5/QtGui/5.12.8 -I/usr/include/aarch64-linux-gnu/qt5/QtGui/5.12.8/QtGui
#include <QApplication>
#include <QWidget>
#include <QLabel>
#include <QFrame>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QProcess>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QStorageInfo>
#include <QWindow>

#ifdef USE_WAYLAND
#include <qpa/qplatformnativeinterface.h>
#include <wayland-client-protocol.h>
#endif

static const QSize SCREEN_SIZE(2160, 1080);
static const char *CALIB_JSON = "/dev/shm/c3calib.json";   // c3calib, every second
static const char *DET_JSON = "/dev/shm/c3det/det.json";   // c3lights, every frame (idle: every second)
static const char *ARCHIVE = "/data/media/0/c4_archive";

static const char *GREEN = "#3ddc84", *YELLOW = "#ffc53d", *RED = "#ff5c5c", *GREY = "#6b7a8c";

static QJsonObject readJson(const char *path) {
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly)) return {};
  return QJsonDocument::fromJson(f.readAll()).object();
}

static QString readText(const QString &path) {
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly)) return {};
  return QString(f.readAll()).trimmed();
}

// hottest CPU / GPU sensor; the lmh-dcvs zones are limits (always 75000), not temperatures
static double cpuTemp() {
  double best = 0;
  for (const QString &z : QDir("/sys/class/thermal").entryList({"thermal_zone*"})) {
    const QString type = readText("/sys/class/thermal/" + z + "/type");
    if (!(type.startsWith("cpu") || type.startsWith("gpu") || type.startsWith("kryo"))) continue;
    best = qMax(best, readText("/sys/class/thermal/" + z + "/temp").toDouble() / 1000.);
  }
  return best;
}

static QLabel *mk(const QString &text, const QString &obj) {
  QLabel *l = new QLabel(text);
  l->setObjectName(obj);
  l->setWordWrap(true);
  return l;
}

static void setColored(QLabel *l, const QString &text, const char *color) {
  l->setText(text);
  l->setStyleSheet(QString("color: %1;").arg(color));
}

struct Card {
  QFrame *frame;
  QLabel *main, *sub;
};

static Card mkCard(const QString &title) {
  Card c;
  c.frame = new QFrame;
  c.frame->setObjectName("card");
  QVBoxLayout *v = new QVBoxLayout(c.frame);
  v->setContentsMargins(36, 26, 36, 26);
  v->setSpacing(10);
  v->addWidget(mk(title, "cap"));
  c.main = mk("—", "big");
  v->addWidget(c.main);
  c.sub = mk("", "sub");
  c.sub->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  v->addWidget(c.sub, 1);
  return c;
}

static QString calibWhy(const QString &why) {
  if (why == "no data yet") return "還沒收到 C4 資料";
  if (why == "too little data") return "資料還不夠";
  if (why.startsWith("overlap")) return "兩台資料重疊不到 30 秒";
  if (why.startsWith("moving")) return "直線行駛 " + why.mid(7);
  if (why == "no aEgo variation" || why.startsWith("aEgo spread")) return "要有加減速才算得出前方";
  return why;
}

int main(int argc, char *argv[]) {
  QString shotPath;
  for (int i = 1; i < argc - 1; i++) {
    if (QString(argv[i]) == "--shot") shotPath = argv[i + 1];
  }
  QApplication app(argc, argv);

  QWidget win;
  win.setObjectName("root");
  win.setFixedSize(SCREEN_SIZE);
  win.setStyleSheet(R"(
    QWidget { background: transparent; color: #e6edf3; font-family: "Noto Sans TC", "Inter", sans-serif; }
    QWidget#root { background: #05080c; }
    QFrame#card { background: #0e141c; border: 2px solid #1b2530; border-radius: 22px; }
    QLabel#title { font-size: 52px; font-weight: 700; color: #7cc4ff; }
    QLabel#clock { font-size: 64px; font-weight: 600; color: #cfe3ff; }
    QLabel#cap { font-size: 34px; color: #6b7a8c; }
    QLabel#big { font-size: 76px; font-weight: 700; }
    QLabel#sub { font-size: 38px; color: #c3ccd6; }
    QLabel#foot { font-size: 32px; color: #5c6773; }
  )");

  QVBoxLayout *root = new QVBoxLayout(&win);
  root->setContentsMargins(48, 32, 48, 28);
  root->setSpacing(26);

  QHBoxLayout *head = new QHBoxLayout;
  head->addWidget(mk("C3 檢測機", "title"));
  head->addStretch(1);
  QLabel *clock = mk("--:--:--", "clock");
  clock->setWordWrap(false);
  head->addWidget(clock);
  root->addLayout(head);

  QGridLayout *grid = new QGridLayout;
  grid->setSpacing(26);
  Card temp = mkCard("溫度"), net = mkCard("網路"), link = mkCard("C4 連線");
  Card cal = mkCard("鏡頭標定"), lights = mkCard("看燈"), arch = mkCard("行車紀錄備份");
  grid->addWidget(temp.frame, 0, 0);
  grid->addWidget(net.frame, 0, 1);
  grid->addWidget(link.frame, 0, 2);
  grid->addWidget(cal.frame, 1, 0);
  grid->addWidget(lights.frame, 1, 1);
  grid->addWidget(arch.frame, 1, 2);
  for (int i = 0; i < 3; i++) grid->setColumnStretch(i, 1);
  for (int i = 0; i < 2; i++) grid->setRowStretch(i, 1);
  root->addLayout(grid, 1);

  QLabel *foot = mk("SSH  comma@—  -p 8022", "foot");
  root->addWidget(foot);

  // ---------- every second: files only (cheap) ----------
  auto tick = [=]() {
    clock->setText(QDateTime::currentDateTime().toString("HH:mm:ss"));

    const double t = cpuTemp();
    setColored(temp.main, QString("%1 °C").arg(t, 0, 'f', 0), t < 70 ? GREEN : (t < 80 ? YELLOW : RED));
    temp.sub->setText(t < 70 ? "正常" : (t < 80 ? "偏熱" : "過熱，會降速"));

    const QJsonObject c = readJson(CALIB_JSON);
    const QJsonObject d = readJson(DET_JSON);
    const bool calOk = !c.isEmpty() && QFileInfo(CALIB_JSON).lastModified().secsTo(QDateTime::currentDateTime()) < 5;
    const bool driving = calOk && c.value("driving").toBool();

    // USB link: cable carrier, then whether C4 is sending carState (it only does while onroad)
    const bool usbUp = readText("/sys/class/net/usb0/carrier") == "1";
    if (!usbUp) {
      setColored(link.main, "線沒接上", RED);
      link.sub->setText("USB 直連沒有訊號\n（先插 C3 端再插 C4）");
    } else if (driving) {
      setColored(link.main, "開車中", GREEN);
      link.sub->setText("線已接上 10.44.0.1\nC4 正在送車速資料");
    } else {
      setColored(link.main, "已接上", GREEN);
      link.sub->setText("線已接上 10.44.0.1\nC4 沒在開車（沒送資料）");
    }

    if (!calOk) {
      setColored(cal.main, "沒在跑", RED);
      cal.sub->setText("c3calib 沒有輸出");
    } else {
      const QString st = c.value("status").toString();
      const QString using_ = c.value("using").toString();
      const QString use = using_ == "this drive" ? "用本趟結果" : (using_ == "previous drive" ? "先沿用上一趟" : "還沒有任何結果");
      if (st == "ok") setColored(cal.main, "完成", GREEN);
      else if (st == "converging") setColored(cal.main, "收斂中", YELLOW);
      else if (st == "error") setColored(cal.main, "錯誤", RED);
      else setColored(cal.main, "標定中", YELLOW);
      cal.sub->setText(use + "\n" + (st == "error" ? c.value("why").toString() : calibWhy(c.value("why").toString())));
    }

    if (d.isEmpty()) {
      setColored(lights.main, "沒在跑", RED);
      lights.sub->setText("c3lights 沒有輸出");
    } else if (!d.value("err").toString().isEmpty()) {
      setColored(lights.main, "錯誤", RED);
      lights.sub->setText(d.value("err").toString());
    } else if (d.value("idle").toBool()) {
      setColored(lights.main, "待機", GREY);
      lights.sub->setText("停車中不辨識\n開車才會開始");
    } else {
      const QJsonArray ls = c.value("lights").toArray();
      QStringList lines;
      const char *col = GREEN;
      for (const QJsonValue &v : ls) {
        const QJsonObject o = v.toObject();
        const QString cls = o.value("cls").toString();
        lines << QString("%1  %2°").arg(o.value("zh").toString()).arg(o.value("az").toDouble(), 0, 'f', 0);
        if (cls == "light_red") col = RED;
        else if (cls == "light_yellow" && col != RED) col = YELLOW;
      }
      if (ls.isEmpty()) setColored(lights.main, "沒看到燈", GREY);
      else setColored(lights.main, lines.first().section("  ", 0, 0), col);
      lines.prepend(QString("每秒 %1 張・%2 ms").arg(d.value("fps").toDouble(), 0, 'f', 1)
                        .arg(d.value("det_ms").toDouble(), 0, 'f', 0));
      lights.sub->setText(lines.mid(0, 4).join("\n"));
    }
  };

  // ---------- slower: IP / WiFi name, disk, last archive copy ----------
  auto pollNet = [=]() {
    QProcess *p = new QProcess;
    QObject::connect(p, static_cast<void (QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished),
                     [=](int, QProcess::ExitStatus) {
                       const QStringList out = QString(p->readAllStandardOutput()).split('\n');
                       p->deleteLater();
                       const QString ip = out.value(0).trimmed(), ssid = out.value(1).trimmed();
                       if (ip.isEmpty()) {
                         setColored(net.main, "沒有 WiFi", RED);
                         net.sub->setText("wlan0 沒有 IP");
                         foot->setText("SSH  (沒有 WiFi IP)");
                       } else {
                         setColored(net.main, ip, "#ffffff");
                         net.sub->setText("WiFi " + (ssid.isEmpty() ? QString("—") : ssid));
                         foot->setText(QString("SSH  comma@%1  -p 8022").arg(ip));
                       }
                     });
    p->start("/bin/bash", {"-c",
                           "ip -4 -o addr show wlan0 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1; "
                           "nmcli -t -f DEVICE,CONNECTION dev 2>/dev/null | grep '^wlan0:' | cut -d: -f2- | sed 's/^openpilot connection //'"});
  };

  auto pollArchive = [=]() {
    QStorageInfo s(ARCHIVE);
    const double total = s.bytesTotal() / 1073741824., used = (s.bytesTotal() - s.bytesFree()) / 1073741824.;  // GiB, as df
    const double pct = total > 0 ? used / total * 100 : 0;
    const int drives = QDir(QString(ARCHIVE) + "/realdata").entryList(QDir::Dirs | QDir::NoDotAndDotDot).size();
    setColored(arch.main, QString("%1 段").arg(drives), drives > 0 ? "#ffffff" : GREY);
    QProcess *p = new QProcess;
    QObject::connect(p, static_cast<void (QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished),
                     [=](int, QProcess::ExitStatus) {
                       QString last = QString(p->readAllStandardOutput()).trimmed();
                       p->deleteLater();
                       // "2026-10-18T15:12:01+0800 host c4_archive[123]: copied 812 MB in 47 s (...)"
                       QString when = last.section(' ', 0, 0).mid(11, 5);
                       QString mb = last.section("copied ", 1, 1).section(' ', 0, 0);
                       QString lastLine = (mb.isEmpty() || when.isEmpty()) ? QString("開機後還沒有新資料")
                                                                           : QString("最後複製 %1（%2 MB）").arg(when, mb);
                       arch.sub->setText(QString("硬碟 %1 / %2 GB（%3%）\n%4")
                                             .arg(used, 0, 'f', 0).arg(total, 0, 'f', 0).arg(pct, 0, 'f', 0).arg(lastLine));
                     });
    p->start("/bin/bash", {"-c", "journalctl -t c4_archive --no-pager -o short-iso 2>/dev/null | grep copied | tail -1"});
  };

  QTimer *t1 = new QTimer(&win);
  QObject::connect(t1, &QTimer::timeout, tick);
  t1->start(1000);
  QTimer *t2 = new QTimer(&win);
  QObject::connect(t2, &QTimer::timeout, pollNet);
  t2->start(10000);
  QTimer *t3 = new QTimer(&win);
  QObject::connect(t3, &QTimer::timeout, pollArchive);
  t3->start(30000);
  tick();
  pollNet();
  pollArchive();

  if (!shotPath.isEmpty()) {
    QTimer::singleShot(2000, [&]() {
      win.grab().save(shotPath);
      app.quit();
    });
    win.show();
    return app.exec();
  }

  win.show();
#ifdef USE_WAYLAND
  if (QApplication::platformName().contains("wayland")) {
    QPlatformNativeInterface *native = QGuiApplication::platformNativeInterface();
    wl_surface *s = reinterpret_cast<wl_surface *>(native->nativeResourceForWindow("surface", win.windowHandle()));
    if (s) {
      wl_surface_set_buffer_transform(s, WL_OUTPUT_TRANSFORM_270);
      wl_surface_commit(s);
    }
    win.setWindowState(Qt::WindowFullScreen);
    win.setVisible(true);
  }
#endif
  return app.exec();
}
