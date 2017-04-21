if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeSoftPaddlePlant
    SOURCE_FILES soft_paddle_plant.cc soft_paddle_state_vector.cc
#    mirror_law_system.cc
    soft_paddle_state_to_bot_visualizer.cc)
    #soft_paddle_poincare_map.cc)
  target_link_libraries(drakeSoftPaddlePlant drakeSystemFramework drakeRBM)
  drake_install_libraries(drakeSoftPaddlePlant)
  drake_install_headers(
    soft_paddle_plant.h
    soft_paddle_plant-inl.h
    soft_paddle_state_vector.h
#    mirror_law_system.h
    soft_paddle_state_to_bot_visualizer.h
    soft_paddle_poincare_map.h
    soft_paddle_poincare_map-inl.h
  )
  drake_install_pkg_config_file(drake-soft-paddle-plant
    TARGET drakeSoftPaddlePlant
    LIBS -ldrakeSoftPaddlePlant
    REQUIRES
      eigen3)

  add_executable(soft_paddle_run_dynamics soft_paddle_run_dynamics.cc)
  target_link_libraries(soft_paddle_run_dynamics
    drakeSoftPaddlePlant
    drakeRigidBodyPlant
    drakeSystemAnalysis)
  drake_add_test(NAME soft_paddle_run_dynamics
    COMMAND soft_paddle_run_dynamics)

#  add_executable(soft_paddle_mirror_law_test soft_paddle_mirror_law_test.cc)
#  target_link_libraries(soft_paddle_mirror_law_test
#    drakeSoftPaddlePlant
#    drakeRigidBodyPlant
#    drakeSystemAnalysis)
#  drake_add_test(NAME soft_paddle_mirror_law_test
#    COMMAND soft_paddle_mirror_law_test)

#  add_executable(soft_paddle_poincare_map_test soft_paddle_poincare_map_test.cc)
#  target_link_libraries(soft_paddle_poincare_map_test drakeSoftPaddlePlant)
#  drake_add_test(NAME soft_paddle_poincare_map_test
#    COMMAND soft_paddle_poincare_map_test)
endif()

#if(BUILD_TESTING)
#  add_subdirectory(test)
#endif()

