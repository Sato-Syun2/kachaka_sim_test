cmake_minimum_required(VERSION 3.8)
project(rear_obstacle_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. auto機能の呼び出し
find_package(ament_cmake_auto REQUIRED)

# 2. package.xml を読んで、必要なパッケージを「すべて自動で」find_packageする魔法
ament_auto_find_build_dependencies()

# 3. 実行ファイルの作成＆依存関係のリンク＆インストールまで「すべて自動で」やってくれる魔法
ament_auto_add_executable(detector_node src/detector_node.cpp)
ament_auto_add_executable(evasion_node src/evasion_node.cpp)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Pythonスクリプトを実行可能ファイルとしてインストール
install(PROGRAMS scripts/dummy_joy.py
  DESTINATION lib/${PROJECT_NAME}
)

# 4. パッケージの最終処理
ament_auto_package()