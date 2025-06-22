#!/bin/bash
counter=0
common_example_dir="example_common"
while [ ${counter} -lt 5 ]
do
    if [ ! -d ${common_example_dir} ];then
        break
    fi
    counter=$((counter=counter + 1))
    cd ..
done

if [ "${counter}" -ge 5 ];then
    echo "${common_example_dir} not found in upper directories!"
    exit 1
fi

folder_list=(
  "./bsp_hosted"
  "./example_common"
)

cmake_fmt="cmake-format"
file_selectors="-iname CMakeLists.txt"
if command -v ${cmake_fmt} &> /dev/null; then
  echo "Auto-formatting all CMakeLists.txt files"
  ${cmake_fmt} -i CMakeLists.txt
  for dir in ${folder_list[@]}; do
    find ${dir} ${file_selectors} | xargs ${cmake_fmt} -i
  done
else
  echo "No ${cmake_fmt} tool found, not formatting CMake files"
fi

cpp_format="clang-format"
file_selectors="-iname *.h -o -iname *.cpp -o -iname *.c -o -iname *.tpp"
if command -v ${cpp_format} &> /dev/null; then
  for dir in ${folder_list[@]}; do
    echo "Auto-formatting C/C++ files in ${dir} recursively"
    find ${dir} ${file_selectors} | xargs ${cpp_format} --style=file -i
  done
else
  echo "No ${cpp_format} tool found, not formatting C++/C files"
fi
