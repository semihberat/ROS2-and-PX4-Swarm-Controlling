#include "communication.hpp"

/** @brief Timer callback for periodic neighbor data publishing */
void NeighborsListener::timer_callback()
{
    this->publish_neighbors_info();
}