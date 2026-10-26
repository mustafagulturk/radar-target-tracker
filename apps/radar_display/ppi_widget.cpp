#include "ppi_widget.hpp"

#include <QConicalGradient>
#include <QFontMetricsF>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QRadialGradient>
#include <QWheelEvent>
#include <QtGlobal>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "radar/coordinates.hpp"

namespace radar {
namespace {

constexpr double kMetresPerSecondToKnots = 1.0 / 0.514444;
constexpr double kLeaderTime_s = 60.0;  ///< Velocity leader shows 60 s of travel.
constexpr double kMinRange_m = 2000.0;
constexpr double kMaxRange_m = 400000.0;
constexpr double kBezel_px = 40.0;  ///< Room outside the scope for the bearing scale.
constexpr double kPickRadius_px = 14.0;
constexpr double kTrailSpan = 0.22;  ///< Phosphor trail length as a fraction of a turn.

// Palette: phosphor green graticule, amber raw video, cyan tracks.
const QColor kWindowBg(6, 10, 8);
const QColor kScopeCentre(9, 30, 18);
const QColor kScopeEdge(3, 13, 8);
const QColor kGrid(70, 200, 120, 60);
const QColor kGridStrong(90, 220, 140, 150);
const QColor kScale(110, 230, 150);
const QColor kSweep(150, 255, 190);
const QColor kPlot(255, 176, 32);
const QColor kTrack(64, 232, 255);
const QColor kTentative(150, 205, 175);
const QColor kSelected(255, 226, 90);
const QColor kHud(120, 225, 155);
const QColor kHudDim(80, 160, 110);

QFont monoFont(int pixel_size, bool bold = false) {
    QFont font(QStringLiteral("DejaVu Sans Mono"));
    font.setStyleHint(QFont::Monospace);
    font.setPixelSize(pixel_size);
    font.setBold(bold);
    return font;
}

QColor withAlpha(QColor color, double alpha) {
    color.setAlpha(static_cast<int>(std::lround(std::clamp(alpha, 0.0, 1.0) * 255.0)));
    return color;
}

/// Picks a "nice" ring spacing that gives four to six rings.
double ringStep(double range_m) {
    static constexpr double kSteps[] = {500.0,   1000.0,  2000.0,  2500.0,  5000.0,
                                        10000.0, 20000.0, 25000.0, 50000.0, 100000.0};
    for (double step : kSteps) {
        if (range_m / step <= 6.0 + 1e-9) {
            return step;
        }
    }
    return kSteps[std::size(kSteps) - 1];
}

QString formatKm(double metres) {
    const double km = metres / 1000.0;
    return std::abs(km - std::round(km)) < 1e-6 ? QString::number(static_cast<int>(std::round(km)))
                                                : QString::number(km, 'f', 1);
}

QString formatBearing(double rad) {
    return QStringLiteral("%1").arg(radToDeg(normalizeAngle(rad)), 5, 'f', 1, QLatin1Char('0'));
}

QString trackLabel(std::uint32_t id) {
    return QStringLiteral("T%1").arg(id, 3, 10, QLatin1Char('0'));
}

}  // namespace

PpiWidget::PpiWidget(QWidget* parent) : QWidget(parent) {
    setAttribute(Qt::WA_OpaquePaintEvent);
    setMouseTracking(false);
    setCursor(Qt::CrossCursor);
    setFocusPolicy(Qt::ClickFocus);
}

void PpiWidget::setSnapshot(PipelineSnapshot snapshot) {
    snapshot_ = std::move(snapshot);
    update();
}

void PpiWidget::setSelectedTrack(std::uint32_t id) {
    if (id != selected_id_) {
        selected_id_ = id;
        update();
    }
}

void PpiWidget::setDisplayRange(double range_m) {
    range_m = std::clamp(range_m, kMinRange_m, kMaxRange_m);
    if (range_m == range_m_) {
        return;
    }
    range_m_ = range_m;
    layoutScope();
    background_dirty_ = true;
    update();
    Q_EMIT displayRangeChanged(range_m_);
}

QPointF PpiWidget::toScreen(double x, double y) const {
    return {center_.x() + x * scale_, center_.y() - y * scale_};
}

void PpiWidget::layoutScope() {
    center_ = QPointF(width() / 2.0, height() / 2.0);
    radius_px_ = std::max(40.0, std::min(width(), height()) / 2.0 - kBezel_px);
    scale_ = radius_px_ / range_m_;
}

void PpiWidget::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    layoutScope();
    background_dirty_ = true;
}

// --- Static graticule --------------------------------------------------------

void PpiWidget::rebuildBackground() {
    const qreal dpr = devicePixelRatioF();
    background_ = QPixmap(size() * dpr);
    background_.setDevicePixelRatio(dpr);
    background_.fill(kWindowBg);

    QPainter p(&background_);
    p.setRenderHint(QPainter::Antialiasing);
    const double r = radius_px_;

    // Scope face with a faint glow towards the centre.
    QRadialGradient face(center_, r);
    face.setColorAt(0.0, kScopeCentre);
    face.setColorAt(1.0, kScopeEdge);
    p.setPen(Qt::NoPen);
    p.setBrush(face);
    p.drawEllipse(center_, r, r);
    p.setBrush(Qt::NoBrush);

    // Bearing spokes every 30 degrees.
    p.setPen(QPen(kGrid, 1.0));
    for (int deg = 0; deg < 360; deg += 30) {
        const double a = degToRad(deg);
        p.drawLine(center_, center_ + QPointF(std::sin(a) * r, -std::cos(a) * r));
    }

    // Range rings, with the outer edge drawn stronger.
    const double step = ringStep(range_m_);
    QPen ring_pen(kGrid, 1.0);
    p.setPen(ring_pen);
    for (double ring = step; ring < range_m_ - 1e-6; ring += step) {
        p.drawEllipse(center_, ring * scale_, ring * scale_);
    }
    // Half-step rings, dashed, when they are far enough apart to read.
    if (step * scale_ > 90.0) {
        QPen dashed(withAlpha(kGrid, 0.12), 1.0, Qt::DashLine);
        p.setPen(dashed);
        for (double ring = step / 2.0; ring < range_m_; ring += step) {
            p.drawEllipse(center_, ring * scale_, ring * scale_);
        }
    }
    p.setPen(QPen(kGridStrong, 1.5));
    p.drawEllipse(center_, r, r);

    // Outer bearing scale: ticks every 5/10/30 degrees and 000..330 labels.
    p.setPen(QPen(kScale, 1.0));
    for (int deg = 0; deg < 360; deg += 5) {
        const double len = deg % 30 == 0 ? 11.0 : (deg % 10 == 0 ? 7.0 : 3.5);
        const double a = degToRad(deg);
        const QPointF dir(std::sin(a), -std::cos(a));
        p.setPen(QPen(withAlpha(kScale, deg % 10 == 0 ? 0.9 : 0.5), deg % 30 == 0 ? 1.6 : 1.0));
        p.drawLine(center_ + dir * r, center_ + dir * (r + len));
    }
    p.setFont(monoFont(12, true));
    p.setPen(kScale);
    for (int deg = 0; deg < 360; deg += 30) {
        const double a = degToRad(deg);
        const QPointF pos = center_ + QPointF(std::sin(a), -std::cos(a)) * (r + 25.0);
        const QRectF box(pos.x() - 20.0, pos.y() - 8.0, 40.0, 16.0);
        p.drawText(box, Qt::AlignCenter, QStringLiteral("%1").arg(deg, 3, 10, QLatin1Char('0')));
    }

    // Range labels just inside each ring, to the right of the north spoke.
    p.setFont(monoFont(11));
    p.setPen(withAlpha(kScale, 0.75));
    for (double ring = step; ring <= range_m_ + 1e-6; ring += step) {
        const double y = center_.y() - ring * scale_;
        p.drawText(QRectF(center_.x() + 5.0, y + 2.0, 80.0, 14.0), Qt::AlignLeft | Qt::AlignTop,
                   formatKm(ring) + QStringLiteral(" km"));
    }

    // Own-ship / radar position.
    p.setPen(QPen(kGridStrong, 1.2));
    p.drawLine(center_ + QPointF(-6, 0), center_ + QPointF(6, 0));
    p.drawLine(center_ + QPointF(0, -6), center_ + QPointF(0, 6));

    background_dirty_ = false;
}

// --- Dynamic layers ----------------------------------------------------------

void PpiWidget::drawSweep(QPainter& p) const {
    const double az = normalizeAngle(snapshot_.antenna_azimuth_rad);
    const double r = radius_px_;

    // QConicalGradient runs counter-clockwise on screen from its start angle
    // (degrees from 3 o'clock), i.e. towards smaller bearings: exactly where
    // the phosphor afterglow sits behind a clockwise-turning beam.
    QConicalGradient trail(center_, 90.0 - radToDeg(az));
    trail.setColorAt(0.0, withAlpha(kSweep, 0.34));
    trail.setColorAt(0.02, withAlpha(kSweep, 0.16));
    trail.setColorAt(kTrailSpan * 0.4, withAlpha(kSweep, 0.06));
    trail.setColorAt(kTrailSpan, withAlpha(kSweep, 0.0));
    trail.setColorAt(1.0, withAlpha(kSweep, 0.0));
    p.setPen(Qt::NoPen);
    p.setBrush(trail);
    p.drawEllipse(center_, r, r);
    p.setBrush(Qt::NoBrush);

    const QPointF tip = center_ + QPointF(std::sin(az), -std::cos(az)) * r;
    p.setPen(QPen(withAlpha(kSweep, 0.18), 6.0, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(center_, tip);
    p.setPen(QPen(withAlpha(kSweep, 0.95), 1.6, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(center_, tip);
}

void PpiWidget::drawPlots(QPainter& p) const {
    if (snapshot_.plots.empty()) {
        return;
    }
    const double persistence_us = std::max(plot_persistence_s_, 0.1) * 1e6;
    p.setPen(Qt::NoPen);
    for (const TimedPlot& tp : snapshot_.plots) {
        const double age_us = snapshot_.sensor_time_us >= tp.timestamp_us
                                  ? static_cast<double>(snapshot_.sensor_time_us - tp.timestamp_us)
                                  : 0.0;
        const double life = 1.0 - age_us / persistence_us;
        if (life <= 0.0) {
            continue;
        }
        const Vec2 pos = polarToCartesian(tp.plot.range_m, tp.plot.azimuth_rad);
        const QPointF s = toScreen(pos[0], pos[1]);
        const double fade = life * life;
        if (life > 0.85) {  // Fresh echo: soft halo.
            p.setBrush(withAlpha(kPlot, 0.25 * fade));
            p.drawEllipse(s, 4.5, 4.5);
        }
        p.setBrush(withAlpha(kPlot, 0.15 + 0.85 * fade));
        p.drawEllipse(s, 2.0, 2.0);
    }
}

void PpiWidget::drawTrack(QPainter& p, const TrackState& t) const {
    const bool confirmed = t.status == TrackStatus::Confirmed;
    const bool selected = t.id == selected_id_;
    const QColor colour = selected ? kSelected : (confirmed ? kTrack : kTentative);
    const QPointF pos = toScreen(t.x, t.y);

    if (!confirmed && !selected) {
        p.setPen(QPen(withAlpha(colour, 0.45), 1.0));
        p.setBrush(Qt::NoBrush);
        p.drawEllipse(pos, 3.5, 3.5);
        return;
    }

    // History trail, fading towards the oldest point.
    const std::size_t n = t.history.size();
    if (n > 0) {
        QPointF prev = toScreen(t.history.front().x, t.history.front().y);
        for (std::size_t i = 1; i <= n; ++i) {
            const QPointF next = i < n ? toScreen(t.history[i].x, t.history[i].y) : pos;
            const double alpha = 0.08 + 0.55 * static_cast<double>(i) / static_cast<double>(n);
            p.setPen(QPen(withAlpha(colour, alpha), 1.3, Qt::SolidLine, Qt::RoundCap));
            p.drawLine(prev, next);
            prev = next;
        }
    }

    // Velocity leader: where the track will be in kLeaderTime_s.
    const QPointF lead = toScreen(t.x + t.vx * kLeaderTime_s, t.y + t.vy * kLeaderTime_s);
    p.setPen(QPen(withAlpha(colour, 0.85), 1.3));
    p.drawLine(pos, lead);

    // Symbol.
    const double h = selected ? 6.5 : 5.0;
    p.setPen(QPen(colour, selected ? 2.0 : 1.6));
    p.setBrush(withAlpha(colour, 0.18));
    p.drawRect(QRectF(pos.x() - h, pos.y() - h, 2 * h, 2 * h));
    p.setBrush(Qt::NoBrush);

    if (selected) {  // Corner brackets around the selected track.
        const double b = 13.0;
        const double c = 5.0;
        p.setPen(QPen(kSelected, 1.4));
        for (int sx : {-1, 1}) {
            for (int sy : {-1, 1}) {
                const QPointF k(pos.x() + sx * b, pos.y() + sy * b);
                p.drawLine(k, k - QPointF(sx * c, 0));
                p.drawLine(k, k - QPointF(0, sy * c));
            }
        }
    }

    // Label in the quadrant opposite the leader so the two never overlap.
    const QFont font = monoFont(12, true);
    p.setFont(font);
    const QString label = trackLabel(t.id);
    const double tw = QFontMetricsF(font).horizontalAdvance(label);
    const double gap = selected ? 17.0 : 10.0;
    const double lx = t.vx > 0.0 ? pos.x() - gap - tw : pos.x() + gap;
    const double ly = t.vy > 0.0 ? pos.y() + gap + 10.0 : pos.y() - gap + 1.0;
    // Labels of tracks near the rim may spill onto the bezel rather than be cut.
    const QPointF from_centre = pos - center_;
    const bool inside = std::hypot(from_centre.x(), from_centre.y()) <= radius_px_;
    p.save();
    p.setClipping(p.hasClipping() && !inside);
    p.setPen(colour);
    p.drawText(QPointF(lx, ly), label);
    p.restore();
}

void PpiWidget::drawInfoBox(QPainter& p, const TrackState& t) const {
    const QPointF pos = toScreen(t.x, t.y);
    const QFont font = monoFont(12);
    const QFontMetricsF fm(font);
    const QString lines[] = {
        trackLabel(t.id) + QStringLiteral("  ") + QString::fromLatin1(toString(t.status)).toUpper(),
        QStringLiteral("RNG %1 km  BRG %2°")
            .arg(t.range() / 1000.0, 6, 'f', 2)
            .arg(formatBearing(t.azimuth())),
        QStringLiteral("SPD %1 kn  HDG %2°")
            .arg(t.speed() * kMetresPerSecondToKnots, 6, 'f', 0)
            .arg(formatBearing(t.heading())),
        QStringLiteral("σ   %1 m   HITS %2").arg(t.position_sigma_m, 5, 'f', 0).arg(t.hits),
    };
    double w = 0.0;
    for (const QString& line : lines) {
        w = std::max(w, fm.horizontalAdvance(line));
    }
    const double line_h = fm.height() + 2.0;
    const QSizeF box_size(w + 20.0, line_h * std::size(lines) + 12.0);

    // Prefer lower right of the symbol; flip when it would leave the widget.
    QPointF corner = pos + QPointF(28.0, 22.0);
    if (corner.x() + box_size.width() > width() - 4.0) {
        corner.setX(pos.x() - 28.0 - box_size.width());
    }
    if (corner.y() + box_size.height() > height() - 4.0) {
        corner.setY(pos.y() - 22.0 - box_size.height());
    }
    const QRectF box(corner, box_size);

    const QPointF anchor(box.center().x() < pos.x() ? box.right() : box.left(),
                         box.center().y() < pos.y() ? box.bottom() : box.top());
    p.setPen(QPen(withAlpha(kSelected, 0.6), 1.0));
    p.drawLine(pos, anchor);
    p.setBrush(QColor(8, 14, 10, 225));
    p.setPen(QPen(withAlpha(kSelected, 0.8), 1.0));
    p.drawRect(box);
    p.setBrush(Qt::NoBrush);

    p.setFont(font);
    double y = box.top() + 6.0 + fm.ascent();
    for (std::size_t i = 0; i < std::size(lines); ++i) {
        p.setPen(i == 0 ? kSelected : withAlpha(kSelected, 0.85));
        p.drawText(QPointF(box.left() + 10.0, y), lines[i]);
        y += line_h;
    }
}

void PpiWidget::drawOverlay(QPainter& p) const {
    const QFont font = monoFont(12);
    const QFont bold = monoFont(12, true);
    const double line_h = QFontMetricsF(font).height() + 3.0;

    std::size_t confirmed = 0;
    for (const TrackState& t : snapshot_.tracks) {
        confirmed += t.status == TrackStatus::Confirmed ? 1 : 0;
    }

    // Top left: display mode and scale.
    double y = 22.0;
    p.setFont(bold);
    p.setPen(kHud);
    p.drawText(QPointF(14.0, y), QStringLiteral("PPI  NORTH UP"));
    p.setFont(font);
    p.setPen(kHudDim);
    y += line_h;
    p.drawText(QPointF(14.0, y), QStringLiteral("RANGE  %1 km").arg(formatKm(range_m_)));
    y += line_h;
    p.drawText(QPointF(14.0, y), QStringLiteral("RINGS  %1 km").arg(formatKm(ringStep(range_m_))));

    // Top right: track picture summary.
    const auto right = [&](double yy, const QString& text) {
        p.drawText(QRectF(0.0, yy - line_h + 4.0, width() - 14.0, line_h),
                   Qt::AlignRight | Qt::AlignVCenter, text);
    };
    y = 22.0;
    p.setFont(bold);
    p.setPen(kHud);
    right(y, QStringLiteral("TRACKS  %1 / %2").arg(confirmed).arg(snapshot_.tracks.size()));
    p.setFont(font);
    p.setPen(kHudDim);
    y += line_h;
    right(y, QStringLiteral("PLOTS  %1").arg(snapshot_.plots.size()));
    y += line_h;
    right(y, QStringLiteral("LEADER  %1 s").arg(static_cast<int>(kLeaderTime_s)));

    // Bottom left: antenna and sensor clock.
    const std::uint64_t t_ms = snapshot_.sensor_time_us / 1000;
    const QString clock = QStringLiteral("%1:%2:%3.%4")
                              .arg(t_ms / 3600000, 2, 10, QLatin1Char('0'))
                              .arg(t_ms / 60000 % 60, 2, 10, QLatin1Char('0'))
                              .arg(t_ms / 1000 % 60, 2, 10, QLatin1Char('0'))
                              .arg(t_ms / 100 % 10);
    y = height() - 14.0 - line_h;
    p.drawText(QPointF(14.0, y),
               QStringLiteral("ANT    %1°").arg(formatBearing(snapshot_.antenna_azimuth_rad)));
    y += line_h;
    p.drawText(QPointF(14.0, y), QStringLiteral("T+     %1").arg(clock));

    // Bottom right: legend.
    const double lx = width() - 132.0;
    y = height() - 14.0 - 2.0 * line_h;
    const double sym_x = lx + 6.0;
    const double dy = -4.0;
    p.setPen(QPen(kTrack, 1.6));
    p.drawRect(QRectF(sym_x - 4.0, y + dy - 4.0, 8.0, 8.0));
    p.setPen(kHudDim);
    p.drawText(QPointF(lx + 20.0, y), QStringLiteral("CONFIRMED"));
    y += line_h;
    p.setPen(QPen(withAlpha(kTentative, 0.6), 1.0));
    p.drawEllipse(QPointF(sym_x, y + dy), 3.5, 3.5);
    p.setPen(kHudDim);
    p.drawText(QPointF(lx + 20.0, y), QStringLiteral("TENTATIVE"));
    y += line_h;
    p.setPen(Qt::NoPen);
    p.setBrush(kPlot);
    p.drawEllipse(QPointF(sym_x, y + dy), 2.2, 2.2);
    p.setBrush(Qt::NoBrush);
    p.setPen(kHudDim);
    p.drawText(QPointF(lx + 20.0, y), QStringLiteral("RAW PLOT"));

    if (snapshot_.stats.batches_processed == 0) {
        p.setFont(monoFont(16, true));
        p.setPen(withAlpha(kSelected, 0.9));
        p.drawText(QRectF(center_.x() - 200.0, center_.y() + 30.0, 400.0, 24.0), Qt::AlignCenter,
                   QStringLiteral("AWAITING RADAR DATA"));
    }
}

void PpiWidget::paintEvent(QPaintEvent* /*event*/) {
    if (background_dirty_ || background_.size() != size() * devicePixelRatioF()) {
        rebuildBackground();
    }

    QPainter p(this);
    p.drawPixmap(0, 0, background_);
    p.setRenderHint(QPainter::Antialiasing);

    QPainterPath scope;
    scope.addEllipse(center_, radius_px_, radius_px_);
    p.setClipPath(scope);

    if (snapshot_.stats.batches_processed > 0) {
        drawSweep(p);
    }
    drawPlots(p);

    const TrackState* selected = nullptr;
    for (const TrackState& t : snapshot_.tracks) {  // Tentative underneath confirmed.
        if (t.status != TrackStatus::Confirmed && t.id != selected_id_) {
            drawTrack(p, t);
        }
    }
    for (const TrackState& t : snapshot_.tracks) {
        if (t.id == selected_id_) {
            selected = &t;
        } else if (t.status == TrackStatus::Confirmed) {
            drawTrack(p, t);
        }
    }
    if (selected != nullptr) {
        drawTrack(p, *selected);
    }

    p.setClipping(false);
    if (selected != nullptr) {
        drawInfoBox(p, *selected);
    }
    drawOverlay(p);
}

// --- Interaction -------------------------------------------------------------

void PpiWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() != Qt::LeftButton) {
        QWidget::mousePressEvent(event);
        return;
    }
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    const QPointF click = event->position();
#else
    const QPointF click = event->localPos();
#endif
    std::uint32_t best_id = 0;
    double best = std::numeric_limits<double>::max();
    for (const TrackState& t : snapshot_.tracks) {
        const QPointF d = toScreen(t.x, t.y) - click;
        double dist = std::hypot(d.x(), d.y());
        if (t.status != TrackStatus::Confirmed) {
            dist += 6.0;  // Prefer confirmed tracks when symbols overlap.
        }
        if (dist < kPickRadius_px && dist < best) {
            best = dist;
            best_id = t.id;
        }
    }
    setSelectedTrack(best_id);
    Q_EMIT trackClicked(best_id);
}

void PpiWidget::wheelEvent(QWheelEvent* event) {
    const double steps = event->angleDelta().y() / 120.0;
    if (steps != 0.0) {
        setDisplayRange(range_m_ * std::pow(0.85, steps));
    }
    event->accept();
}

}  // namespace radar
