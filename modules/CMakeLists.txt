#add subdirectories containing modules here

set(modules_built_by_default "utils" "stereo" "segmentation")

add_subdirectory("python")
add_subdirectory("datapipe")
add_subdirectory("utils")
add_subdirectory("stereo")
add_subdirectory("segmentation")
add_subdirectory("calib")
add_subdirectory("test")