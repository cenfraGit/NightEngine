# test_shader_parity.py
# NightMaterialDefault and NightMaterialTexture carry near-duplicate GLSL.
# Every lighting change so far (shadows, blinn-phong, point light, the
# point-light array) had to be written twice, so this test fails if the
# shared lighting functions ever drift apart.
#
# It is a guard, not an endorsement: unifying the two shaders is the real
# fix, but the duplication is cheap to police and expensive to refactor
# while working examples depend on it.
#
# run:  python tests/test_shader_parity.py

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT = os.path.join(ROOT, "NightEngine", "Materials", "NightMaterialDefault.py")
TEXTURE = os.path.join(ROOT, "NightEngine", "Materials", "NightMaterialTexture.py")

# functions that must be byte-identical in both shaders
SHARED_FUNCTIONS = ["CalcLightDir", "CalcLightPoint", "CalcShadow"]

# declarations that must appear identically in both
SHARED_DECLARATIONS = [
    "#define MAX_POINT_LIGHTS 8",
    "uniform int point_light_count;",
    "uniform LightPoint point_lights[MAX_POINT_LIGHTS];",
    "for (int i = 0; i < point_light_count && i < MAX_POINT_LIGHTS; ++i) {",
]


def read(path):
    with open(path, encoding="utf-8") as handle:
        return handle.read()


def extract_function(source, name):
    """pulls one GLSL function body out by brace matching."""
    match = re.search(rf"^\s*\w[\w\s\*]*\b{re.escape(name)}\s*\([^;)]*\)\s*\{{",
                      source, re.MULTILINE)
    assert match, f"could not locate function {name}"
    start = match.start()
    depth = 0
    for index in range(match.end() - 1, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                body = source[start:index + 1]
                # normalise indentation only; everything else must match
                return "\n".join(line.strip() for line in body.splitlines())
    raise AssertionError(f"unbalanced braces in {name}")


def test_shared_lighting_functions_are_identical():
    default, texture = read(DEFAULT), read(TEXTURE)
    for name in SHARED_FUNCTIONS:
        a = extract_function(default, name)
        b = extract_function(texture, name)
        assert a == b, (
            f"{name}() has drifted between NightMaterialDefault and "
            f"NightMaterialTexture.\n--- Default ---\n{a}\n--- Texture ---\n{b}")


def test_shared_declarations_present_in_both():
    default, texture = read(DEFAULT), read(TEXTURE)
    for declaration in SHARED_DECLARATIONS:
        assert declaration in default, f"missing from Default: {declaration}"
        assert declaration in texture, f"missing from Texture: {declaration}"


def test_point_light_array_size_matches_python_constant():
    from NightEngine.NightBase import MAX_POINT_LIGHTS
    for path in (DEFAULT, TEXTURE):
        source = read(path)
        match = re.search(r"#define MAX_POINT_LIGHTS (\d+)", source)
        assert match, f"no MAX_POINT_LIGHTS define in {os.path.basename(path)}"
        assert int(match.group(1)) == MAX_POINT_LIGHTS, (
            f"{os.path.basename(path)} declares MAX_POINT_LIGHTS="
            f"{match.group(1)} but NightBase.MAX_POINT_LIGHTS="
            f"{MAX_POINT_LIGHTS}")


def test_no_stale_single_light_uniform_remains():
    """the old scalar uniform must be gone from both shaders, or a stale
    set_uniform call would silently do nothing."""
    for path in (DEFAULT, TEXTURE):
        source = read(path)
        assert "uniform LightPoint light_point;" not in source, (
            f"{os.path.basename(path)} still declares the old single "
            f"light_point uniform")


if __name__ == "__main__":
    tests = [(name, obj) for name, obj in sorted(globals().items())
             if name.startswith("test_") and callable(obj)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except Exception as e:
            failed += 1
            print(f"  FAIL  {name}: {type(e).__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
