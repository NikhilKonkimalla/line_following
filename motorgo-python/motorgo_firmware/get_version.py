# Script to run at build time to get the config for the config file

import ast

Import("projenv")
env = DefaultEnvironment()

def crc16(data: bytes, polynomial: int = 0x1021, init_crc: int = 0xFFFF) -> int:
    crc = init_crc
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ polynomial
            else:
                crc <<= 1
            crc &= 0xFFFF  # Ensure CRC remains 16-bit
    return crc

setup_path = "../setup.py"

# Load and parse the setup.py file
with open(setup_path, 'r') as f:
    setup_contents = f.read()

# Parse the setup.py contents into an AST
setup_ast = ast.parse(setup_contents)

# Initialize version to None
version = None

# Walk through the AST to find the 'version' argument in the setup() call
for node in ast.walk(setup_ast):
    if isinstance(node, ast.Call) and getattr(node.func, 'id', '') == 'setup':
        for keyword in node.keywords:
            if keyword.arg == 'version':
                if isinstance(keyword.value, ast.Str):
                    version = keyword.value.s
                elif hasattr(ast, 'Constant') and isinstance(keyword.value, ast.Constant):
                    version = keyword.value.value
                break

# Raise an error if version is not found
if version is None:
    version = "0.0.0"
    version_hash = "0x0000"
else:
    # Generate a version hash
    version_hash = crc16(version.encode())

    # Convert it into a hex string
    version_hash = f"0x{version_hash:04X}"

# Print in green, found version
print("\033[92m" + f"Found version {version}\tHash: {version_hash}" + "\033[0m")

# Set the version as an environment variable
projenv.Append(CPPDEFINES=[("VERSION_HASH", version_hash)])
env.Append(CPPDEFINES=[("VERSION_HASH", version_hash)])
