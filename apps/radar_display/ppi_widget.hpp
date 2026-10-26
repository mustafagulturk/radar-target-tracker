#pragma once

#include <QPixmap>
#include <QWidget>
#include <cstdint>

#include "radar/tracking_pipeline.hpp"

namespace radar {

/// Plan position indicator: north-up radar scope showing the rotating sweep
/// with its phosphor trail, decaying raw plots and tracks with history trails
/// and velocity leaders.
///
/// Screen mapping: x (east) grows to the right, y (north) grows upwards;
/// bearings are measured clockwise from north. Static graticule is cached in
/// a pixmap and only redrawn on resize or zoom.
class PpiWidget : public QWidget {
    Q_OBJECT

public:
    explicit PpiWidget(QWidget* parent = nullptr);

    void setSnapshot(PipelineSnapshot snapshot);

    /// Track drawn highlighted with an info box; 0 clears the selection.
    void setSelectedTrack(std::uint32_t id);
    std::uint32_t selectedTrack() const { return selected_id_; }

    void setDisplayRange(double range_m);
    double displayRange() const { return range_m_; }

    /// Age (seconds of sensor time) at which raw plots have fully faded.
    void setPlotPersistence(double seconds) { plot_persistence_s_ = seconds; }

    QSize sizeHint() const override { return {900, 860}; }
    QSize minimumSizeHint() const override { return {360, 360}; }

Q_SIGNALS:
    /// The user clicked on the scope; id is the nearest track, or 0 if none.
    void trackClicked(std::uint32_t id);
    void displayRangeChanged(double range_m);

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    QPointF toScreen(double x, double y) const;
    void layoutScope();
    void rebuildBackground();

    void drawSweep(QPainter& painter) const;
    void drawPlots(QPainter& painter) const;
    void drawTrack(QPainter& painter, const TrackState& track) const;
    void drawInfoBox(QPainter& painter, const TrackState& track) const;
    void drawOverlay(QPainter& painter) const;

    PipelineSnapshot snapshot_;
    QPixmap background_;
    QPointF center_;
    double radius_px_ = 1.0;
    double scale_ = 1.0;  ///< Pixels per metre.
    double range_m_ = 40000.0;
    double plot_persistence_s_ = 4.0;
    std::uint32_t selected_id_ = 0;
    bool background_dirty_ = true;
};

}  // namespace radar
