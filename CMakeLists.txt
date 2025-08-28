cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Fiat600Display)

# In ESP-IDF 5.x non serve nulla in più per includere bt se è nella cartella components
# ma PlatformIO talvolta richiede che il componente sia esplicitamente aggiunto:
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/components/bt)
