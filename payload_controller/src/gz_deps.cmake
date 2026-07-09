function(link_gz_transport target)
    find_package(gz-transport13 REQUIRED)
    find_package(gz-msgs10 REQUIRED)
    target_link_libraries(${target} PRIVATE
        gz-transport13::gz-transport13
        gz-msgs10::gz-msgs10
    )
endfunction()
