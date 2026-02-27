#!/bin/bash
echo "Current directory: $(pwd)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROTO_DIR="$SCRIPT_DIR/proto-public-api"
PYTHON_OUT_DIR="$SCRIPT_DIR/hex_device/hex_device_py/generated"

# Create generated folder if it doesn't exist
if [ ! -d "$PYTHON_OUT_DIR" ]; then
    mkdir -p "$PYTHON_OUT_DIR"
    echo "Created directory: $PYTHON_OUT_DIR"
fi

echo "Proto directory: $PROTO_DIR"
echo "Python output directory: $PYTHON_OUT_DIR"
protoc -I="$PROTO_DIR" --python_out="$PYTHON_OUT_DIR" "$PROTO_DIR"/*.proto

# Fix imports to use relative imports
echo "Fixing imports to use relative imports..."
sed -i.bak 's/^import public_api_types_pb2/from . import public_api_types_pb2/g' "$PYTHON_OUT_DIR"/*_pb2.py
rm -f "$PYTHON_OUT_DIR"/*.bak

# Create __init__.py to make it a Python package
if [ ! -f "$PYTHON_OUT_DIR/__init__.py" ]; then
    touch "$PYTHON_OUT_DIR/__init__.py"
    echo "Created __init__.py in $PYTHON_OUT_DIR"
fi

echo "Done!"