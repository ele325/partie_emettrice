cmake_minimum_required(VERSION 3.16)

# Cette ligne dit à ESP-IDF d'utiliser ton fichier personnalisé
set(PARTITION_TABLE_CSV_PATH "${CMAKE_CURRENT_SOURCE_DIR}/partitions.csv")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(capteur_irrigation)