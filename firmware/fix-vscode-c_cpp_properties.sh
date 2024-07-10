#!/bin/sh
sed -i -e 's/\\(/\(/g' .vscode/c_cpp_properties.json
sed -i -e 's/\\)/\)/g' .vscode/c_cpp_properties.json
