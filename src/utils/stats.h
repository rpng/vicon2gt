/**
 * MIT License
 * Copyright (c) 2018 Patrick Geneva @ University of Delaware (Robot Perception & Navigation Group)
 * Copyright (c) 2018 Kevin Eckenhoff @ University of Delaware (Robot Perception & Navigation Group)
 * Copyright (c) 2018 Guoquan Huang @ University of Delaware (Robot Perception & Navigation Group)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef STATS_H
#define STATS_H

#include <Eigen/Eigen>


/**
 * @brief Statistics object for a given set scalar time series values.
 *
 * Ensure that you call the calculate() function to update the values before using them.
 * This will compute all the final results from the values in @ref values vector.
 */
struct Stats {

public:

    /// Root mean squared for the given values
    double rmse = 0.0;

    /// Mean of the given values
    double mean = 0.0;

    /// Median of the given values
    double median = 0.0;

    /// Standard deviation of given values
    double std = 0.0;

    /// Max of the given values
    double max = 0.0;

    /// Min of the given values
    double min = 0.0;

    /// 99th percentile
    double ninetynine = 0.0;

    /// Timestamp when these values occured at
    std::vector<double> timestamps;

    /// Values (e.g. error or nees at a given time)
    std::vector<double> values;

    /// Bound of these values (e.g. our expected covariance bound)
    std::vector<double> values_bound;


    /// Will calculate all values from our vectors of information
    void calculate() {

        // Sort the data for easy finding of values
        std::vector<double> values_sorted = values;
        std::sort(values_sorted.begin(), values_sorted.end());

        // If we don't have any data, just return :(
        if(values_sorted.empty())
            return;

        // Now that its been sorted, can easily grab min and max
        min = values_sorted.at(0);
        max = values_sorted.at(values_sorted.size()-1);

        // Compute median
        // ODD:  grab middle from the sorted vector
        // EVEN: average the middle two numbers
        if (values_sorted.size()==1) {
            median = values_sorted.at(values_sorted.size()-1);
        } else if(values_sorted.size() % 2 == 1) {
            median = values_sorted.at(values_sorted.size() / 2);
        } else if(values_sorted.size() > 1) {
            median = 0.5 * (values_sorted.at(values_sorted.size()/2-1) + values_sorted.at(values_sorted.size()/2));
        } else {
            median = 0.0;
        }

        // Compute mean and rmse
        mean = 0;
        for (size_t i = 0; i < values_sorted.size(); i++) {
            assert(!std::isnan(values_sorted.at(i)));
            mean += values_sorted.at(i);
            rmse += values_sorted.at(i) * values_sorted.at(i);
        }
        mean /= values_sorted.size();
        rmse = std::sqrt(rmse / values_sorted.size());

        // Using mean, compute standard deviation
        std = 0;
        for (size_t i = 0; i < values_sorted.size(); i++) {
            std += std::pow(values_sorted.at(i) - mean, 2);
        }
        std = std::sqrt(std / (values_sorted.size() - 1));

        // 99th percentile
        // TODO: is this correct?
        // TODO: http://sphweb.bumc.bu.edu/otlt/MPH-Modules/BS/BS704_Probability/BS704_Probability10.html
        ninetynine = mean + 2.326*std;

    }

    /// Will clear any old values
    void clear() {
        timestamps.clear();
        values.clear();
        values_bound.clear();
    }

};

#endif //STATS_H