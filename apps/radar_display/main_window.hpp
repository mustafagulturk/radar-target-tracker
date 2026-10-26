#pragma once

#include <QElapsedTimer>
#include <QMainWindow>
#include <cstdint>
#include <vector>

#include "radar/tracking_pipeline.hpp"

class QCheckBox;
class QLabel;
class QTableWidget;
class QTimer;

namespace radar {

class PpiWidget;

/// Operator console: PPI scope in the centre, track table on the right and
/// pipeline health in the status bar. Polls the pipeline snapshot at ~30 Hz.
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(TrackingPipeline& pipeline, double display_range_m, QWidget* parent = nullptr);

    /// Selects the confirmed track with the most hits. Returns false if none exists.
    bool selectEstablishedTrack();

private:
    void refresh();
    void updateTable(const PipelineSnapshot& snapshot);
    void updateDetails(const PipelineSnapshot& snapshot);
    void updateStatus(const PipelineSnapshot& snapshot);
    void selectTrack(std::uint32_t id);
    void onTableSelectionChanged();

    QWidget* buildSidePanel();
    void buildStatusBar();

    TrackingPipeline& pipeline_;
    PipelineSnapshot last_snapshot_;
    std::uint32_t selected_id_ = 0;
    std::vector<std::uint32_t> row_ids_;

    PpiWidget* ppi_ = nullptr;
    QTableWidget* table_ = nullptr;
    QCheckBox* show_tentative_ = nullptr;
    QLabel* details_ = nullptr;
    QTimer* timer_ = nullptr;

    QLabel* link_label_ = nullptr;
    QLabel* datagrams_label_ = nullptr;
    QLabel* lost_label_ = nullptr;
    QLabel* rejected_label_ = nullptr;
    QLabel* queue_label_ = nullptr;
    QLabel* batch_label_ = nullptr;
    QLabel* tracks_label_ = nullptr;
    QLabel* range_label_ = nullptr;

    std::uint64_t last_datagrams_ = 0;
    QElapsedTimer since_last_datagram_;
};

}  // namespace radar
