#pragma once

namespace sim_time {
    /**
     * @brief Set the global time.
     *
     * This time is used by simulation and can read from ``sim_time::now()``
     *
     * @param time current time in seconds
     */
    void setTime(double time);

    /**
     * @brief Get the current time in seconds
     *
     * @return double time in seconds
     */
    double now();
}
