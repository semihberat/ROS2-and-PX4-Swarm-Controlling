#include "autonomus.hpp"

// Mission state machine - called every 100ms
void SwarmMemberPathPlanner::state_cycle_callback()
{
    if (this->current_neighbors_info_ == nullptr)
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for neighbors info...");
        return;
    }

    switch (this->current_mission)
    {
    case Mission::FORMATIONAL_TAKEOFF:

        formational_takeoff();
        break;
    case Mission::FORMATIONAL_ROTATION:
        formational_rotation();
        break;
    case Mission::GOTO_POSITION:
        goto_position();
        break;
    case Mission::DO_PROCESS:
        do_process();
        break;
    case Mission::END_TASK:
        end_task();
        break;
    }
}

void SwarmMemberPathPlanner::collision_avoidance()
{
    if (this->current_neighbors_info_ == nullptr || this->initial_n_distances.empty())
        return;

    current_n_distances = autonomus_utils::all_distances(this->current_neighbors_info_->neighbor_positions, this->current_neighbors_info_->main_position);

    collision_bias.vlat = 0.0;
    collision_bias.vlon = 0.0;

    // Dinamik Çarpışma Eşiği (Hıza bağlı, min 1.5m mesafe)
    auto v = std::sqrt(current_commands.v_lat * current_commands.v_lat + current_commands.v_lon * current_commands.v_lon);
    double d_col_threshold = std::max(1.5, v * 0.5);

    // Çekim (Formasyon Koruma) Sabiti - Formalite Düzeyine Çekildi
    double k_form = 0.05; // Çok daha zayıf, tatlı bir hizalama (eski 0.2)
    double k_rep = 1.0;   // Sadece dib dibe gelindiğinde sert ikaz verecek itki (eski 0.8)

    for (size_t i = 0; i < this->current_n_distances.size(); ++i)
    {
        double current_dist = this->current_n_distances[i].distance;

        // 1. İTİCİ GÜÇ (Repulsive - Sadece çok yaklaşıldığında ve gerçekten tehlike varsa sertleşecek şekilde)
        if (current_dist < d_col_threshold)
        {
            double safe_dist = std::max(0.1, current_dist);
            // Üstel olarak artan bir itki (Exponential Repulsion): Ne kadar yaklaşırsan o kadar şiddetli iter.
            // (d_col_threshold - safe_dist) değeri küçüldükçe karesi alınarak çok dibe girmediği sürece hafif kalması sağlandı.
            double dist_diff = d_col_threshold - safe_dist;
            double force_magnitude = k_rep * (dist_diff * dist_diff); // Üstel itki

            // Yönü komşunun tam zıttına (tersine) çeviriyoruz
            double dir_lat = -this->current_n_distances[i].dlat_meter / safe_dist;
            double dir_lon = -this->current_n_distances[i].dlon_meter / safe_dist;

            collision_bias.vlat += dir_lat * force_magnitude;
            collision_bias.vlon += dir_lon * force_magnitude;
        }
        else
        {
            // 2. ÇEKİCİ / HİZALAYICI GÜÇ (Formaliteden - Yavaş Formasyon Düzeltmesi)
            double dlat_diff = this->current_n_distances[i].dlat_meter - this->initial_n_distances[i].dlat_meter;
            double dlon_diff = this->current_n_distances[i].dlon_meter - this->initial_n_distances[i].dlon_meter;

            // Ölü bant devasa yapıldı: 1.5 metre. Yani bayağı dağılmadıkları sürece formasyon düzeltmek için uğraşmayacak.
            if (std::abs(dlat_diff) > 1.5)
            {
                collision_bias.vlat += (dlat_diff > 0 ? dlat_diff - 1.5 : dlat_diff + 1.5) * k_form;
            }

            if (std::abs(dlon_diff) > 1.5)
            {
                collision_bias.vlon += (dlon_diff > 0 ? dlon_diff - 1.5 : dlon_diff + 1.5) * k_form;
            }
        }
    }

    // Stabilite için sınırları tatlı bir aralıkta tutuyoruz
    collision_bias.vlat = std::clamp<double>(collision_bias.vlat, -1.0, 1.0);
    collision_bias.vlon = std::clamp<double>(collision_bias.vlon, -1.0, 1.0);
}
