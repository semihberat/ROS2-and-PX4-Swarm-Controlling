#!/bin/bash
echo "Path planner'lar konfigüre ediliyor..."
for i in {1..3}; do ros2 lifecycle set /path_planner_$i configure & done
wait
echo "Path planner'lar aktive ediliyor..."
for i in {1..3}; do ros2 lifecycle set /path_planner_$i activate & done
wait
echo "Tüm path_planner düğümleri başarıyla aktive edildi!"
