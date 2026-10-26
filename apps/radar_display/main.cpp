// Radar display: runs the receive/track pipeline and shows the air picture on
// a PPI scope with a live track table. Optionally saves a screenshot and exits
// (used to produce the README image under xvfb).

#include <QApplication>
#include <QImage>
#include <QMessageBox>
#include <QPalette>
#include <QStyleFactory>
#include <QTimer>
#include <algorithm>
#include <iostream>
#include <string>

#include "main_window.hpp"
#include "radar/cli.hpp"
#include "radar/tracking_pipeline.hpp"

namespace {

void printUsage() {
    std::cout << "Usage: radar_display [options]\n"
                 "  --port <n>               UDP port to listen on        (default 5005)\n"
                 "  --range <m>              display range in metres      (default 40000)\n"
                 "  --screenshot <file.png>  save the window to a PNG and exit; the most\n"
                 "                           established track is selected for the picture\n"
                 "  --screenshot-after <ms>  delay before the screenshot  (default 8000)\n";
}

void applyDarkTheme(QApplication& app) {
    app.setStyle(QStyleFactory::create(QStringLiteral("Fusion")));
    QPalette p;
    const QColor window(18, 22, 20);
    const QColor base(10, 14, 12);
    const QColor text(200, 220, 208);
    const QColor accent(40, 120, 80);
    p.setColor(QPalette::Window, window);
    p.setColor(QPalette::WindowText, text);
    p.setColor(QPalette::Base, base);
    p.setColor(QPalette::AlternateBase, QColor(16, 22, 19));
    p.setColor(QPalette::ToolTipBase, window);
    p.setColor(QPalette::ToolTipText, text);
    p.setColor(QPalette::Text, text);
    p.setColor(QPalette::Button, QColor(26, 32, 29));
    p.setColor(QPalette::ButtonText, text);
    p.setColor(QPalette::BrightText, QColor(255, 90, 80));
    p.setColor(QPalette::Highlight, accent);
    p.setColor(QPalette::HighlightedText, QColor(255, 255, 255));
    p.setColor(QPalette::Link, QColor(90, 200, 140));
    p.setColor(QPalette::Mid, QColor(40, 48, 44));
    p.setColor(QPalette::Dark, QColor(8, 10, 9));
    p.setColor(QPalette::Disabled, QPalette::Text, QColor(100, 110, 104));
    p.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(100, 110, 104));
    app.setPalette(p);
}

bool saveScreenshot(QWidget& window, const QString& path) {
    const qreal dpr = window.devicePixelRatioF();
    QImage image(window.size() * dpr, QImage::Format_ARGB32_Premultiplied);
    image.setDevicePixelRatio(dpr);
    image.fill(Qt::black);
    window.render(&image);
    return image.save(path, "PNG");
}

}  // namespace

int main(int argc, char** argv) {
    QApplication app(argc, argv);  // Strips Qt's own options (-platform, ...) from argv.
    QApplication::setApplicationName(QStringLiteral("radar_display"));

    radar::PipelineConfig config;
    double display_range_m = 40000.0;
    std::string screenshot;
    long screenshot_after_ms = 8000;
    try {
        const radar::cli::Args args(argc, argv);
        if (args.has("help")) {
            printUsage();
            return 0;
        }
        config.port = static_cast<std::uint16_t>(args.get("port", 5005L));
        display_range_m = args.get("range", display_range_m);
        screenshot = args.get("screenshot", std::string());
        screenshot_after_ms = std::max(0L, args.get("screenshot-after", screenshot_after_ms));
    } catch (const std::exception& e) {
        std::cerr << "radar_display: " << e.what() << '\n';
        printUsage();
        return 2;
    }

    applyDarkTheme(app);

    radar::TrackingPipeline pipeline(config);
    try {
        pipeline.start();
    } catch (const std::exception& e) {
        const QString message =
            QStringLiteral("Cannot listen on UDP port %1:\n%2").arg(config.port).arg(e.what());
        std::cerr << "radar_display: " << message.toStdString() << '\n';
        if (screenshot.empty()) {  // Never block an unattended run on a dialog.
            QMessageBox::critical(nullptr, QStringLiteral("Radar display"), message);
        }
        return 1;
    }
    std::cout << "Listening for radar plots on UDP port " << pipeline.port() << '\n';

    radar::MainWindow window(pipeline, display_range_m);
    window.show();

    if (!screenshot.empty()) {
        const int after = static_cast<int>(std::min(screenshot_after_ms, 3600000L));
        // Select a track shortly before the capture so the info box is painted.
        QTimer::singleShot(std::max(0, after - 500), &window,
                           [&window] { window.selectEstablishedTrack(); });
        QTimer::singleShot(after, &window, [&window, &app, &screenshot] {
            const QString path = QString::fromStdString(screenshot);
            const bool ok = saveScreenshot(window, path);
            std::cerr << (ok ? "Saved screenshot to " : "Failed to save screenshot to ")
                      << screenshot << '\n';
            app.exit(ok ? 0 : 1);
        });
    }

    const int rc = QApplication::exec();
    pipeline.stop();
    return rc;
}
