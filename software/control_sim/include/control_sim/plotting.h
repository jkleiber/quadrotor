
#pragma once

#include "implot.h"

namespace Plotting
{

// utility structure for realtime plot
class ScrollingBuffer
{

public:
    ScrollingBuffer(std::string name = "ScrollPlot", std::string label = "",
                    bool is_shaded = false, int max_size = 20000)
    {
        // Memory
        max_size_ = max_size;
        offset_ = 0;
        data_.reserve(max_size_);

        // Axes
        y_min_ = -1;
        y_max_ = 1;

        // Plotting features
        name_ = name;
        label_ = label;
        history_ = 10.0;
        is_shaded_ = is_shaded;
    }
    void Init(std::string name, std::string label, float history = 10.0,
              bool is_shaded = false)
    {
        name_ = name;
        label_ = label;
        history_ = history;
        is_shaded_ = is_shaded;
    }
    void SetYLimits(float ymin, float ymax)
    {
        y_min_ = ymin;
        y_max_ = ymax;
    }
    void AddPoint(float x, float y)
    {
        if (data_.size() < max_size_)
            data_.push_back(ImVec2(x, y));
        else
        {
            data_[offset_] = ImVec2(x, y);
            offset_ = (offset_ + 1) % max_size_;
        }
    }
    void Reset()
    {
        Erase();
        data_.reserve(max_size_);
    }
    void Erase()
    {
        if (data_.size() > 0)
        {
            data_.shrink(0);
            offset_ = 0;
        }
    }
    void Plot(float t)
    {
        // Flags
        ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

        // Plot
        if (ImPlot::BeginPlot(name_.c_str(), ImVec2(-1, 150)))
        {
            ImPlot::SetupAxes(NULL, NULL, flags, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1, t - history_, t,
                                    ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, y_min_, y_max_);
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);

            if (is_shaded_)
            {
                ImPlot::PlotShaded(label_.c_str(), &data_[0].x, &data_[0].y,
                                   data_.size(), -INFINITY, offset_,
                                   2 * sizeof(float));
            }
            else
            {
                ImPlot::PlotLine(label_.c_str(), &data_[0].x, &data_[0].y,
                                 data_.size(), offset_, 2 * sizeof(float));
            }

            ImPlot::EndPlot();
        }
    }

private:
    // Timing
    float t_;

    // Vector and sizing
    ImVector<ImVec2> data_;
    int max_size_;
    int offset_;

    // Axes
    float y_min_;
    float y_max_;
    std::string label_;

    // Plotting features
    std::string name_;
    float history_;
    bool is_shaded_;
};



// Graph
class DynamicGraph {
    public:
    DynamicGraph(std::string name = "ScrollPlot", std::string label = "",
                    bool is_shaded = false, int max_size = 20000)
    {
        // Memory
        max_size_ = max_size;
        offset_ = 0;
        data_.reserve(max_size_);

        // Axes
        y_min_ = -1;
        y_max_ = 1;
        x_min_ = -1;
        x_max_ = 1;

        // Plotting features
        name_ = name;
        label_ = label;
    }
    void Init(std::string name, std::string label, float history = 10.0,
              bool is_shaded = false)
    {
        name_ = name;
        label_ = label;
    }
    void SetXLimits(float xmin, float xmax)
    {
        x_min_ = xmin;
        x_max_ = xmax;
    }
    void SetYLimits(float ymin, float ymax)
    {
        y_min_ = ymin;
        y_max_ = ymax;
    }
    void AddPoint(float x, float y)
    {
        if (data_.size() < max_size_)
            data_.push_back(ImVec2(x, y));
        else
        {
            data_[offset_] = ImVec2(x, y);
            offset_ = (offset_ + 1) % max_size_;
        }
    }
    void Reset()
    {
        Erase();
        data_.reserve(max_size_);
    }
    void Erase()
    {
        if (data_.size() > 0)
        {
            data_.shrink(0);
            offset_ = 0;
        }
    }
    void Plot()
    {
        // Flags
        ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

        // Plot
        if (ImPlot::BeginPlot(name_.c_str(), ImVec2(300, 300)))
        {
            ImPlot::SetupAxes(NULL, NULL, flags, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1, x_min_, x_max_);
            ImPlot::SetupAxisLimits(ImAxis_Y1, y_min_, y_max_);
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);

            // Plot a line
            ImPlot::PlotLine(label_.c_str(), &data_[0].x, &data_[0].y,
                             data_.size(), offset_, 2 * sizeof(float));


            ImPlot::EndPlot();
        }
    }

private:

    // Vector and sizing
    ImVector<ImVec2> data_;
    int max_size_;
    int offset_;

    // Axes
    float y_min_;
    float y_max_;
    float x_min_;
    float x_max_;
    std::string label_;

    // Plotting features
    std::string name_;
};

} // namespace Plotting