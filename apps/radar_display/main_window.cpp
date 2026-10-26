#include "main_window.hpp"

#include <QCheckBox>
#include <QHeaderView>
#include <QLabel>
#include <QSignalBlocker>
#include <QSplitter>
#include <QStatusBar>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>

#include "ppi_widget.hpp"
#include "radar/coordinates.hpp"

namespace radar {
namespace {

constexpr double kMetresPerSecondToKnots = 1.0 / 0.514444;
constexpr int kRefreshIntervalMs = 33;
constexpr qint64 kLinkTimeoutMs = 1500;

enum Column { ColId, ColStatus, ColRange, ColBearing, ColSpeed, ColHeading, ColCount };

const QColor kConfirmedText(64, 232, 255);
const QColor kTentativeText(130, 160, 145);

QFont monoFont(int point_size, bool bold = false) {
    QFont font(QStringLiteral("DejaVu Sans Mono"));
    font.setStyleHint(QFont::Monospace);
    font.setPointSize(point_size);
    font.setBold(bold);
    return font;
}

QString trackLabel(std::uint32_t id) {
    return QStringLiteral("T%1").arg(id, 3, 10, QLatin1Char('0'));
}

QString degrees(double rad) {
    return QStringLiteral("%1").arg(radToDeg(normalizeAngle(rad)), 5, 'f', 1, QLatin1Char('0'));
}

}  // namespace

MainWindow::MainWindow(TrackingPipeline& pipeline, double display_range_m, QWidget* parent)
    : QMainWindow(parent), pipeline_(pipeline) {
    setWindowTitle(QStringLiteral("Radar Target Tracker - PPI  [UDP %1]").arg(pipeline_.port()));

    ppi_ = new PpiWidget(this);
    ppi_->setDisplayRange(display_range_m);

    auto* splitter = new QSplitter(Qt::Horizontal, this);
    splitter->addWidget(ppi_);
    splitter->addWidget(buildSidePanel());
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 0);
    splitter->setSizes({960, 440});
    splitter->setChildrenCollapsible(false);
    setCentralWidget(splitter);

    buildStatusBar();

    connect(ppi_, &PpiWidget::trackClicked, this, &MainWindow::selectTrack);
    connect(ppi_, &PpiWidget::displayRangeChanged, this, [this](double range_m) {
        range_label_->setText(QStringLiteral("RANGE %1 km").arg(range_m / 1000.0, 0, 'f', 1));
    });

    since_last_datagram_.start();
    timer_ = new QTimer(this);
    timer_->setTimerType(Qt::PreciseTimer);
    connect(timer_, &QTimer::timeout, this, &MainWindow::refresh);
    timer_->start(kRefreshIntervalMs);

    resize(1400, 900);
    refresh();
}

QWidget* MainWindow::buildSidePanel() {
    auto* panel = new QWidget(this);
    auto* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(8, 10, 10, 8);
    layout->setSpacing(8);

    auto* title = new QLabel(QStringLiteral("TRACK TABLE"), panel);
    title->setFont(monoFont(11, true));
    title->setStyleSheet(QStringLiteral("color: #78e19b; letter-spacing: 2px;"));
    layout->addWidget(title);

    table_ = new QTableWidget(0, ColCount, panel);
    table_->setHorizontalHeaderLabels({QStringLiteral("TRACK"), QStringLiteral("STATUS"),
                                       QStringLiteral("RNG km"), QStringLiteral("BRG °"),
                                       QStringLiteral("SPD kn"), QStringLiteral("HDG °")});
    table_->setFont(monoFont(10));
    table_->horizontalHeader()->setFont(monoFont(9, true));
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table_->horizontalHeader()->setHighlightSections(false);
    table_->verticalHeader()->setVisible(false);
    table_->verticalHeader()->setDefaultSectionSize(24);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_->setFocusPolicy(Qt::NoFocus);
    table_->setShowGrid(false);
    table_->setAlternatingRowColors(true);
    connect(table_, &QTableWidget::itemSelectionChanged, this,
            &MainWindow::onTableSelectionChanged);
    layout->addWidget(table_, 1);

    show_tentative_ = new QCheckBox(QStringLiteral("Show tentative tracks"), panel);
    show_tentative_->setStyleSheet(QStringLiteral(
        "QCheckBox::indicator { width: 12px; height: 12px; border: 1px solid #4f8a66;"
        " background: #0a110d; }"
        "QCheckBox::indicator:checked { background: #3fb97a; }"));
    connect(show_tentative_, &QCheckBox::toggled, this, [this] { updateTable(last_snapshot_); });
    layout->addWidget(show_tentative_);

    auto* details_title = new QLabel(QStringLiteral("SELECTED TRACK"), panel);
    details_title->setFont(monoFont(11, true));
    details_title->setStyleSheet(QStringLiteral("color: #78e19b; letter-spacing: 2px;"));
    layout->addWidget(details_title);

    details_ = new QLabel(panel);
    details_->setFont(monoFont(10));
    details_->setTextFormat(Qt::PlainText);
    details_->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    details_->setMinimumHeight(118);
    details_->setStyleSheet(
        QStringLiteral("QLabel { color: #ffe25a; background: #0a110d; border: 1px solid #23402e;"
                       " padding: 8px; }"));
    layout->addWidget(details_);

    panel->setMinimumWidth(380);
    return panel;
}

void MainWindow::buildStatusBar() {
    const auto make = [this](const QString& tip) {
        auto* label = new QLabel(this);
        label->setFont(monoFont(9));
        label->setToolTip(tip);
        label->setContentsMargins(6, 0, 6, 0);
        statusBar()->addPermanentWidget(label);
        return label;
    };
    link_label_ = make(QStringLiteral("Data link state"));
    datagrams_label_ = make(QStringLiteral("UDP datagrams received"));
    lost_label_ = make(QStringLiteral("Datagrams lost (sequence gaps)"));
    rejected_label_ = make(QStringLiteral("Datagrams that failed to decode"));
    queue_label_ = make(QStringLiteral("Batches waiting for the tracker / dropped on overflow"));
    batch_label_ = make(QStringLiteral("Tracker time spent on the last batch"));
    tracks_label_ = make(QStringLiteral("Confirmed / total tracks"));
    range_label_ = make(QStringLiteral("Display range (mouse wheel to zoom)"));
    range_label_->setText(
        QStringLiteral("RANGE %1 km").arg(ppi_->displayRange() / 1000.0, 0, 'f', 1));
    statusBar()->setSizeGripEnabled(false);
}

void MainWindow::refresh() {
    PipelineSnapshot snapshot = pipeline_.snapshot();
    updateStatus(snapshot);
    updateTable(snapshot);
    updateDetails(snapshot);
    ppi_->setSnapshot(snapshot);
    last_snapshot_ = std::move(snapshot);
}

void MainWindow::updateTable(const PipelineSnapshot& snapshot) {
    std::vector<const TrackState*> rows;
    rows.reserve(snapshot.tracks.size());
    for (const TrackState& t : snapshot.tracks) {
        if (t.status == TrackStatus::Confirmed || show_tentative_->isChecked()) {
            rows.push_back(&t);
        }
    }
    std::sort(rows.begin(), rows.end(), [](const TrackState* a, const TrackState* b) {
        if (a->status != b->status) {
            return a->status == TrackStatus::Confirmed;
        }
        return a->id < b->id;
    });

    // Rebuilding the rows must not look like a user selection change.
    const QSignalBlocker blocker(table_);
    table_->setRowCount(static_cast<int>(rows.size()));
    row_ids_.clear();
    int selected_row = -1;

    for (std::size_t i = 0; i < rows.size(); ++i) {
        const TrackState& t = *rows[i];
        const int row = static_cast<int>(i);
        row_ids_.push_back(t.id);
        if (t.id == selected_id_) {
            selected_row = row;
        }
        const bool confirmed = t.status == TrackStatus::Confirmed;
        const QString cells[ColCount] = {
            trackLabel(t.id),
            confirmed ? QStringLiteral("CONF") : QStringLiteral("TENT"),
            QString::number(t.range() / 1000.0, 'f', 2),
            degrees(t.azimuth()),
            QString::number(t.speed() * kMetresPerSecondToKnots, 'f', 0),
            degrees(t.heading()),
        };
        for (int col = 0; col < ColCount; ++col) {
            QTableWidgetItem* item = table_->item(row, col);
            if (item == nullptr) {
                item = new QTableWidgetItem;
                item->setTextAlignment(col <= ColStatus ? Qt::AlignLeft | Qt::AlignVCenter
                                                        : Qt::AlignRight | Qt::AlignVCenter);
                table_->setItem(row, col, item);
            }
            item->setText(cells[col]);
            item->setForeground(confirmed ? kConfirmedText : kTentativeText);
        }
    }

    if (selected_row >= 0) {
        table_->selectRow(selected_row);
    } else {
        table_->clearSelection();
    }
}

void MainWindow::updateDetails(const PipelineSnapshot& snapshot) {
    const auto it = std::find_if(snapshot.tracks.begin(), snapshot.tracks.end(),
                                 [this](const TrackState& t) { return t.id == selected_id_; });
    if (selected_id_ == 0 || it == snapshot.tracks.end()) {
        details_->setText(selected_id_ == 0
                              ? QStringLiteral("Click a track on the scope\nor a row in the table.")
                              : QStringLiteral("%1 dropped.").arg(trackLabel(selected_id_)));
        return;
    }
    const TrackState& t = *it;
    details_->setText(QStringLiteral("%1   %2\n"
                                     "Position  %3 km E  %4 km N\n"
                                     "Range     %5 km    Bearing %6°\n"
                                     "Speed     %7 m/s  (%8 kn)\n"
                                     "Heading   %9°    Sigma %10 m\n"
                                     "Hits      %11      Misses %12")
                          .arg(trackLabel(t.id), QString::fromLatin1(toString(t.status)).toUpper())
                          .arg(t.x / 1000.0, 7, 'f', 2)
                          .arg(t.y / 1000.0, 7, 'f', 2)
                          .arg(t.range() / 1000.0, 6, 'f', 2)
                          .arg(degrees(t.azimuth()))
                          .arg(t.speed(), 5, 'f', 0)
                          .arg(t.speed() * kMetresPerSecondToKnots, 0, 'f', 0)
                          .arg(degrees(t.heading()))
                          .arg(t.position_sigma_m, 0, 'f', 0)
                          .arg(t.hits, -4)
                          .arg(t.consecutive_misses));
}

void MainWindow::updateStatus(const PipelineSnapshot& snapshot) {
    const PipelineStats& s = snapshot.stats;
    if (s.datagrams_received != last_datagrams_) {
        last_datagrams_ = s.datagrams_received;
        since_last_datagram_.restart();
    }
    const bool live = last_datagrams_ > 0 && since_last_datagram_.elapsed() < kLinkTimeoutMs;
    link_label_->setText(live ? QStringLiteral("● LIVE  UDP %1").arg(pipeline_.port())
                              : QStringLiteral("● NO DATA  UDP %1").arg(pipeline_.port()));
    link_label_->setStyleSheet(live ? QStringLiteral("color: #4ee08a;")
                                    : QStringLiteral("color: #ff6a5a;"));

    datagrams_label_->setText(QStringLiteral("DGRAMS %1").arg(s.datagrams_received));
    lost_label_->setText(QStringLiteral("LOST %1").arg(s.sequence_gaps));
    rejected_label_->setText(QStringLiteral("REJECTED %1").arg(s.datagrams_rejected));
    queue_label_->setText(
        QStringLiteral("QUEUE %1  DROP %2").arg(s.queue_depth).arg(s.batches_dropped));
    batch_label_->setText(QStringLiteral("BATCH %1 µs").arg(s.last_processing_us, 0, 'f', 1));

    std::size_t confirmed = 0;
    for (const TrackState& t : snapshot.tracks) {
        confirmed += t.status == TrackStatus::Confirmed ? 1 : 0;
    }
    tracks_label_->setText(
        QStringLiteral("TRACKS %1 / %2").arg(confirmed).arg(snapshot.tracks.size()));
}

void MainWindow::selectTrack(std::uint32_t id) {
    selected_id_ = id;
    ppi_->setSelectedTrack(id);
    updateTable(last_snapshot_);
    updateDetails(last_snapshot_);
}

void MainWindow::onTableSelectionChanged() {
    const QList<QTableWidgetItem*> items = table_->selectedItems();
    if (items.isEmpty()) {
        return;  // Keep the PPI selection; the track may just be filtered out.
    }
    const auto row = static_cast<std::size_t>(items.front()->row());
    if (row < row_ids_.size()) {
        selectTrack(row_ids_[row]);
    }
}

bool MainWindow::selectEstablishedTrack() {
    const TrackState* best = nullptr;
    for (const TrackState& t : last_snapshot_.tracks) {
        if (t.status == TrackStatus::Confirmed && (best == nullptr || t.hits > best->hits)) {
            best = &t;
        }
    }
    if (best != nullptr) {
        selectTrack(best->id);
    }
    return best != nullptr;
}

}  // namespace radar
