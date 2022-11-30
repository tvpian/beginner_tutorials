cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "/src/first_publisher/src/*.cpp" -e "/src/first_subscriber/src/*.cpp") > results/cppcheck.txt

