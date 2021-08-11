# Clean
rm */*.hpp
rm */*.py

# Make
lcm-gen -xp *.lcm
mkdir -p cpp
mv *.hpp cpp

mkdir -p python
mv *.py python