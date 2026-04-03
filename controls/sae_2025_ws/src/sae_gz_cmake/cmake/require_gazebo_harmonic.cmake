function(sae_require_gazebo_harmonic)
  if("$ENV{GZ_VERSION}" STREQUAL "" AND NOT "$ENV{IGNITION_VERSION}" STREQUAL "")
    message(DEPRECATION
      "Environment variable [IGNITION_VERSION] is deprecated. Use [GZ_VERSION] instead."
    )
    set(ENV{GZ_VERSION} "$ENV{IGNITION_VERSION}")
  endif()

  if("$ENV{GZ_VERSION}" STREQUAL "")
    message(WARNING
      "GZ_VERSION is not set. This workspace targets Gazebo Harmonic. "
      "Set 'export GZ_VERSION=harmonic' before running colcon build. "
      "Continuing with Gazebo Harmonic for this configure step."
    )
    set(ENV{GZ_VERSION} harmonic)
    return()
  endif()

  string(TOLOWER "$ENV{GZ_VERSION}" _sae_gz_version)
  if(NOT _sae_gz_version STREQUAL "$ENV{GZ_VERSION}")
    set(ENV{GZ_VERSION} "${_sae_gz_version}")
  endif()

  if(NOT _sae_gz_version STREQUAL "harmonic")
    message(FATAL_ERROR
      "This workspace targets Gazebo Harmonic, but GZ_VERSION='${_sae_gz_version}'. "
      "Set 'export GZ_VERSION=harmonic' before running colcon build."
    )
  endif()
endfunction()
